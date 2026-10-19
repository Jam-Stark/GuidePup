"""RGB navigation: planner -> navigation model -> CUDA MPPI -> existing WBC.

The coordinator schedules I/O and bounded execution events. It does not judge
semantic stages, edit instructions, or read simulator truth/contact output.
"""
from __future__ import annotations

import asyncio
import base64
import json
from pathlib import Path
import time
import uuid
import subprocess

import aiohttp
import numpy as np
from PIL import Image

from control.gpu_tracking import GPUTrackingMPPI
from .codex_planner import CodexPlanner
from .decision_evidence import NavigationProgress, handoff_history_evidence, model_observation
from .handoff import draw_target_review, target_evidence, view_freshness
from .observation_skills import ObservationSkill
from .pi_planner import PiPlanner
from .planner_contract import CONTRACT, LIMITS, SKILLS, validate_decision


def project_path(local, pose):
    c, s = np.cos(pose[2]), np.sin(pose[2])
    result = np.asarray(local, dtype=float).copy()
    if result.ndim != 2 or result.shape[1] != 3 or not len(result) or not np.isfinite(result).all():
        raise RuntimeError('Navigation model returned an invalid local path')
    result[:, :2] = result[:, :2]@np.array([[c, s], [-s, c]])+pose[:2]
    result[:, 2] += pose[2]
    return np.vstack([pose, result])


def failure_description(failure):
    if isinstance(failure, BaseExceptionGroup):
        return '; '.join(failure_description(item) for item in failure.exceptions)
    return f'{type(failure).__name__}: {failure}'


class RGBNavigation:
    def __init__(self, plant, config, controller_config, output, goal=None):
        self.plant, self.config, self.output, self.goal = plant, config, Path(output), goal
        self.output.mkdir(exist_ok=True)
        self.frames = self.output/'rgb'
        self.frames.mkdir()
        self.controller = GPUTrackingMPPI(controller_config)
        planner_backend = config['planner']['backend']
        if planner_backend == 'codex_exec':
            self.planner = CodexPlanner(config['planner'], self.output/'planner')
        elif planner_backend == 'pi_tools':
            self.planner = PiPlanner(config['planner'], self.output/'planner')
        else:
            raise ValueError(f'Unsupported planner backend {planner_backend!r}')
        self.log = (self.output/'events.jsonl').open('w', buffering=1)
        self.control_log = (self.output/'mppi.jsonl').open('w', buffering=1)
        self.execution_log = (self.output/'execution.jsonl').open('w', buffering=65536)
        self.sensor_log = (self.output/'estimated_state.jsonl').open('w', buffering=65536)
        self.plant.enable_force_stop()
        self.plant.record_sensors(self.output)
        self.plant.record_offline(self.output)
        self.video_log = (self.output/'video.log').open('w')
        self.writer = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-f', 'image2pipe',
            '-vcodec', 'mjpeg', '-framerate', '15', '-i', 'pipe:0', '-an', '-c:v', 'libx264',
            '-preset', 'fast', '-crf', '23', '-pix_fmt', 'yuv420p', str(self.output/'camera.mp4')],
            stdin=subprocess.PIPE, stderr=self.video_log)
        self.phase = 'IDLE'
        self.command = np.zeros(2)
        self.frame = None
        self.frame_event = asyncio.Event()
        self.decision_queue = asyncio.Queue()
        self.done = asyncio.Event()
        self.instruction = None
        self.recoveries = 0
        self.path = None
        self.path_stamp = None
        self.path_model_sequence = None
        self.generation = 0
        self.nav_session = None
        self.latest_model = None
        self.latest_decision = None
        self.decision_history = []
        self.last_action_image = None
        self.last_model_image = None
        self.progress = None
        self.previous_attempts = []
        self.stop_stable_since = None
        self.stop_requested_s = None
        self.stable_stop = False
        self.frame_index = 0
        self.applied_controls = 0
        self.solver_calls = 0
        self.current_control_id = None
        self.last_applied_control_id = None
        self.preview = False
        self.observation = None
        self.observation_history = []
        self.observation_calls = 0
        self.observations_in_cycle = 0
        self.recovery_cycle_active = False
        self.force_stop = None
        self.force_stop_history = []
        self.force_stop_stable_since = None
        self.standing_stable_since = None
        self.pending_navigation = None
        self.handoff_history = []
        self.handoff_output = self.output/'target_handoffs'
        self.handoff_output.mkdir()

    def record(self, kind, **values):
        self.log.write(json.dumps(dict(kind=kind, stamp_s=self.plant.stamp_s,
            wall_time_s=time.time(), **values), ensure_ascii=False, allow_nan=False)+'\n')

    def _transition(self, phase, **evidence):
        before, self.phase = self.phase, phase
        self.record('state', before=before, after=phase, **evidence)

    def _decision_event(self, event, **evidence):
        if self.phase not in ('NAVIGATING', 'IDLE'):
            return
        self._request_planner(event, reset_cycle=True, **evidence)

    def _request_planner(self, event, *, reset_cycle=False, **evidence):
        self.command[:] = 0.
        self.path = None
        self.current_control_id = None
        self.generation += 1
        if reset_cycle:
            self.recovery_cycle_active = False
            self.observations_in_cycle = 0
        self._transition('AWAITING_PLANNER', event=event, evidence=evidence)
        self.decision_queue.put_nowait(dict(event=event, generation=self.generation, **evidence))

    def references(self):
        state = self.plant.robot_state
        return dict(estimated_state=state.payload(),
            estimated_stopped=bool(np.linalg.norm(state.twist_base[:2]) < .08 and abs(state.planar_twist[2]) < .1),
            motion_command_vx_wz=self.command.tolist(), applied_control_count=self.applied_controls,
            standing_wait=dict(force_walking_authorized=self.phase == 'NAVIGATING' and self.path is not None,
                stable_for_s=0. if self.standing_stable_since is None else state.stamp_s-self.standing_stable_since,
                estimated_stable=self.standing_stable_since is not None and
                    state.stamp_s-self.standing_stable_since >= LIMITS['stable_stop_s']),
            force_stop=self.plant.force_stop.payload(),
            geometric_completion='unavailable_rgb_only', collision_checking='unavailable_rgb_only',
            observer_role='Read-only evidence; no stage or completion authority')

    def _external_force_stopped(self, observation, preempted_control_id):
        self._clear_pending_navigation('INTERRUPTED_EXTERNAL_FORCE')
        self.force_stop = dict(trigger=self.plant.force_stop.trigger,
            interrupted_phase=self.phase, instruction=self.instruction,
            preempted_control_id=preempted_control_id,
            interrupted_skill=None if observation is None else dict(
                skill_id=observation.skill_id, request=observation.request),
            navigation_generation_before=self.generation,
            resume_authority='Planner may choose navigation, turn or advance; hold keeps the stop active')
        self.force_stop_history.append(self.force_stop)
        self.force_stop_stable_since = None
        self.command[:] = 0.
        self.path = None
        self.current_control_id = None
        self.generation += 1
        self.recovery_cycle_active = False
        self.observations_in_cycle = 0
        self._transition('AWAITING_PLANNER', event='EXTERNAL_FORCE_STOP', evidence=self.force_stop)

    def _resume_external_force(self, decision):
        if self.plant.force_stop.latched:
            release = self.plant.resume_force_stop(decision['reason'])
            self.force_stop.update(resumed_sensor_time_s=self.plant.stamp_s, resume_proposal=decision)
            self.record('external_force_stop_resumed', proposal=decision, **release)
            self.force_stop = None
            self.force_stop_stable_since = None

    async def _plant_loop(self, preview_seconds):
        dt = self.plant.metadata.control_dt
        next_capture = self.plant.stamp_s
        next_control = self.plant.stamp_s
        started = self.plant.stamp_s
        stalled_since = None
        while not self.done.is_set():
            wall = time.monotonic()
            stamp = self.plant.stamp_s
            state = self.plant.robot_state
            if self.phase == 'NAVIGATING' and self.path is not None and stamp >= next_control-1e-8:
                if stamp-self.path_stamp > self.config['reference_max_age_s']:
                    self._decision_event('STALE_REFERENCE', age_s=stamp-self.path_stamp)
                else:
                    result = self.controller.solve(state, self.path, self.command)
                    self.command = np.asarray(result['command'])
                    self.solver_calls += 1
                    self.current_control_id = self.solver_calls
                    self.control_log.write(json.dumps(dict(stamp_s=stamp, state=state.payload(),
                        control_id=self.current_control_id, generation=self.generation,
                        navigation_model_sequence=self.path_model_sequence,
                        instruction=self.instruction, **result), allow_nan=False)+'\n')
                    next_control = stamp+self.controller.dt
                    speed = np.linalg.norm(state.twist_base[:2])
                    turning = abs(state.planar_twist[2])
                    commanded = np.linalg.norm(self.command) > .1
                    if commanded and speed < .03 and turning < .03:
                        if stalled_since is None:
                            stalled_since = stamp
                        elif stamp-stalled_since >= self.config['stall_window_s']:
                            self._decision_event('NO_ESTIMATED_MOTION', duration_s=stamp-stalled_since)
                    else:
                        stalled_since = None
            else:
                stalled_since = None
            observation = self.observation if self.phase == 'OBSERVING' else None
            if observation is not None:
                self.command = observation.next_command(state, dt)
            elif self.phase != 'NAVIGATING':
                self.command[:] = 0.
            # Navigation uses the accepted first knot; decision/stop states hold zero.
            control_id = self.current_control_id if self.phase == 'NAVIGATING' else None
            requested_command = [float(self.command[0]), 0., float(self.command[1])]
            allow_force_walking = ((self.phase == 'NAVIGATING' and self.path is not None)
                or (observation is not None and observation.stage == 'MOTION'
                    and observation.request['action'] != 'hold_observe'
                    and np.max(np.abs(self.command)) > 1e-8))
            effective = self.plant.step((self.command[0], 0., self.command[1]),
                allow_force_walking=allow_force_walking,
                explicit_motion=observation is not None and np.max(np.abs(self.command)) > 1e-8)
            force_stopped = self.plant.force_stop.latched
            new_force_stop = force_stopped and self.force_stop is None
            preempted_control_id = control_id if force_stopped else None
            if force_stopped:
                if new_force_stop:
                    self._external_force_stopped(observation, preempted_control_id)
                self.command[:] = 0.
                control_id = None
            self.execution_log.write(json.dumps(dict(stamp_before_s=stamp,
                stamp_after_s=self.plant.stamp_s, command_vx_vy_wz=effective.tolist(),
                requested_command_vx_vy_wz=requested_command,
                preempted_control_id=preempted_control_id, wbc=self.plant.last_step,
                control_id=control_id,
                skill_id=observation.skill_id if observation is not None else None,
                source='external_force_stop' if force_stopped else 'observation_skill' if observation is not None else
                    ('mppi' if control_id is not None else 'zero_hold')), allow_nan=False)+'\n')
            if control_id is not None and control_id != self.last_applied_control_id:
                self.applied_controls += 1
                self.last_applied_control_id = control_id
            state = self.plant.robot_state
            estimated_stopped = np.linalg.norm(state.twist_base[:2]) < .08 and abs(state.planar_twist[2]) < .1
            if (self.plant.last_step['standing_hold'] or self.plant.last_step['paused']) and estimated_stopped:
                if self.standing_stable_since is None:
                    self.standing_stable_since = state.stamp_s
            else:
                self.standing_stable_since = None
            if self.progress is not None:
                self.progress.observe(state, effective)
            if observation is not None:
                if new_force_stop:
                    observation.interrupt_force_stop(state, effective, dt, self.force_stop)
                else:
                    observation.observe(state, effective, dt)
            self.sensor_log.write(json.dumps(state.payload(), allow_nan=False)+'\n')
            if self.plant.stamp_s >= next_capture-1e-8 or new_force_stop:
                captured = self.plant.capture(f'rgb_{self.frame_index:06d}')
                path = self.frames/f'{captured.frame_id}.jpg'
                Image.fromarray(captured.rgb).save(path, quality=95, subsampling=0)
                self.writer.stdin.write(path.read_bytes())
                self.frame = dict(frame=captured, path=path, stamp_s=captured.state.stamp_s)
                self.frame_index += 1
                if self.plant.stamp_s >= next_capture-1e-8:
                    next_capture += self.plant.profile['update_period_s']
                self.frame_event.set()
                if observation is not None:
                    observation.sample(self.frame)
                    if observation.finished:
                        result = observation.finish(state, self.frame)
                        self.observation_history.append(result)
                        self.observation = None
                        self.command[:] = 0.
                        if new_force_stop:
                            self.record('observation_force_interrupted', result=result)
                        else:
                            self._transition('AWAITING_PLANNER', event='OBSERVATION_RESULT', result=result)
                            self.decision_queue.put_nowait(dict(event='OBSERVATION_RESULT',
                                generation=self.generation, observation=result))
                if new_force_stop:
                    self.record('external_force_stop', evidence=self.force_stop,
                        frame_id=captured.frame_id, observation_result=None if observation is None else result)
                    self.decision_queue.put_nowait(dict(event='EXTERNAL_FORCE_STOP',
                        generation=self.generation, **self.force_stop))
                if self.frame_index == 1 and not self.preview:
                    self._decision_event('TASK_START')
            if self.phase == 'STOPPING':
                if self.references()['estimated_stopped']:
                    if self.stop_stable_since is None:
                        self.stop_stable_since = self.plant.stamp_s
                    if self.plant.stamp_s-self.stop_stable_since >= .6:
                        self.stable_stop = True
                        self._transition('COMPLETED', evidence='Planner visual-arrival claim and estimated stable stop; physical success not certified')
                        self.done.set()
                else:
                    self.stop_stable_since = None
                if self.plant.stamp_s-self.stop_requested_s > 10. and not self.done.is_set():
                    self._transition('STOP_NOT_CONFIRMED', evidence='Estimated stop did not settle within the local 10 s stop observation')
                    self.done.set()
            if force_stopped:
                if self.references()['estimated_stopped']:
                    if self.force_stop_stable_since is None:
                        self.force_stop_stable_since = self.plant.stamp_s
                else:
                    self.force_stop_stable_since = None
                stable = (self.force_stop_stable_since is not None and
                    self.plant.stamp_s-self.force_stop_stable_since >= .6)
                elapsed = self.plant.stamp_s-self.force_stop['trigger']['command_stop_sensor_time_s']
                self.force_stop.update(estimated_stable_stop=bool(stable),
                    stop_observation_elapsed_s=elapsed)
            if self.preview and self.plant.stamp_s-started >= preview_seconds:
                self._transition('PREVIEW_COMPLETE')
                self.done.set()
            await asyncio.sleep(max(0., dt-(time.monotonic()-wall)))

    def _clear_pending_navigation(self, status):
        if self.pending_navigation is not None:
            self.pending_navigation['record'].update(status=status, closed_at_s=self.plant.stamp_s)
            self.record('target_handoff_closed', handoff=self.pending_navigation['record'])
            self.pending_navigation = None

    def _pending_evidence(self):
        if self.pending_navigation is None:
            return None
        pending = self.pending_navigation
        return dict(handoff_id=pending['record']['handoff_id'], instruction=pending['instruction'],
            target=pending['target'], source_frame_id=pending['frame']['frame'].frame_id,
            pointing=None if 'first_result' not in pending else pending['first_result'].get('pointing'),
            comparison=pending.get('comparison'), first_result=pending.get('first_result'),
            candidate_motion_executed=False,
            decision_authority='GPT chooses confirm_navigation, a revised proposal or another action')

    async def _await_standing_observation(self, generation):
        started = self.plant.stamp_s
        while (generation == self.generation and self.phase == 'AWAITING_PLANNER'
                and not self.references()['standing_wait']['estimated_stable']
                and self.plant.stamp_s-started < LIMITS['settle_timeout_s']):
            await asyncio.sleep(.02)
        self.record('standing_observation', elapsed_s=self.plant.stamp_s-started,
            evidence=self.references()['standing_wait'],
            role='Local settling evidence only; no GPT or whole-task deadline')

    def _propose_navigation(self, result, frame):
        if result['target']['frame_id'] != frame['frame'].frame_id:
            raise ValueError('Planner target frame_id differs from its supplied current RGB')
        self._clear_pending_navigation('REPLACED_BY_PLANNER')
        self._archive_navigation_progress()
        self.instruction = result['instruction']
        self.last_action_image = frame['path']
        self.command[:] = 0.
        self.path = None
        self.current_control_id = None
        self.generation += 1
        record = dict(handoff_id=str(uuid.uuid4()), instruction=result['instruction'],
            target=result['target'], source_frame_id=frame['frame'].frame_id,
            source_capture_s=frame['stamp_s'], proposed_at_s=self.plant.stamp_s,
            source_image=str(frame['path']), status='PROBING_TARGET',
            native_input='Original RGB JPEG and instruction only; no box sent to navigation model')
        self.handoff_history.append(record)
        self.pending_navigation = dict(record=record, frame=frame,
            instruction=result['instruction'], target=result['target'])
        self.record('target_handoff_proposed', handoff=record)
        self._transition('PROBING_TARGET', handoff_id=record['handoff_id'],
            evidence='Standing inference only; no MPPI/reference execution before GPT review')

    def _probe_ready(self, result):
        pending = self.pending_navigation
        if pending is None:
            raise RuntimeError('Target probe completed without its pending proposal')
        comparison = target_evidence(pending['frame'], pending['target'], result)
        pending.update(comparison=comparison, first_result=result)
        overlay = draw_target_review(pending['frame'], comparison,
            self.handoff_output/f'{len(self.handoff_history):03d}_review.jpg')
        pending['overlay'] = overlay
        pending['record'].update(status='AWAITING_TARGET_REVIEW', first_result=result,
            comparison=comparison, overlay=str(overlay), probe_received_s=self.plant.stamp_s)
        self.record('target_handoff_probe', handoff=pending['record'])
        self._request_planner('NAVIGATION_TARGET_REVIEW', handoff=self._pending_evidence())

    def _confirm_navigation(self, result):
        pending = self.pending_navigation
        if pending is None or 'first_result' not in pending:
            raise RuntimeError('confirm_navigation requires a completed pending target probe')
        pending['record'].update(status='CONFIRMED', confirmed_at_s=self.plant.stamp_s,
            confirmation=result, probe_trajectory_executed=False)
        self.record('target_handoff_confirmed', handoff=pending['record'])
        stopped = pending['first_result']['stop']
        self.pending_navigation = None
        self.generation += 1
        if stopped:
            # A reviewed STOP remains a STOP. Do not replace it with motion or
            # recycle/re-time its reference after the planner's thinking interval.
            self._request_planner('MODEL_STOP', model=self.latest_model,
                evidence='GPT confirmed target identity, but the probe itself proposed STOP')
        else:
            self._resume_external_force(result)
            self.progress = NavigationProgress(self.instruction, self.plant.robot_state,
                self.applied_controls, self.solver_calls)
            self.path = None
            self.current_control_id = None
            self._transition('NAVIGATING', instruction=self.instruction,
                handoff_id=pending['record']['handoff_id'],
                history_policy='Same session; obtain a fresh prediction after confirmation. Probe path is never re-timed or executed.')

    def _decision_input(self, event, frame=None):
        frame = self.frame if frame is None else frame
        images = []
        if self.last_action_image is not None:
            images.append(('Previous navigation action start RGB', self.last_action_image))
        if self.last_model_image is not None:
            images.append((self.latest_model['source_image_label'], self.last_model_image))
        images.append((f"Current A2 HEAD RGB; frame {frame['frame'].frame_id}; "
            f"sensor time {frame['stamp_s']:.3f} s", frame['path']))
        if self.pending_navigation is not None and 'overlay' in self.pending_navigation:
            source = self.pending_navigation['frame']
            images.append((f"Target handoff review overlay; EXACT original frame {source['frame'].frame_id}; "
                'green=your selected region; red=model object point; yellow=model action point; '
                'original navigation input unchanged; no candidate movement executed', self.pending_navigation['overlay']))
        for observation in self.observation_history[-2:]:
            images.append((f'Observation {observation["skill_id"]}: {observation["action"]}; '
                f'{observation["start_sensor_time_s"]:.2f} to {observation["end_sensor_time_s"]:.2f} s; '
                'chronological views, estimated angles, NOT a calibrated panorama', observation['contact_sheet']))
        progress = None if self.progress is None else self.progress.snapshot(
            self.plant.robot_state, self.applied_controls, self.solver_calls)
        return dict(task=self.goal, event=event, instruction=self.instruction,
            current_observation=dict(frame_id=frame['frame'].frame_id, capture_time_s=frame['stamp_s'],
                image_size=[frame['frame'].rgb.shape[1], frame['frame'].rgb.shape[0]]),
            pending_navigation=self._pending_evidence(),
            target_handoff_history=handoff_history_evidence(self.handoff_history),
            planner_decision_history=list(self.decision_history),
            completion_mode=self.config['completion'], recoveries_used=self.recoveries,
            references=self.references(),
            navigation_model=self.latest_model, navigation_progress=progress,
            previous_navigation_attempts=list(self.previous_attempts),
            available_actions=CONTRACT['actions'], observation_limits=LIMITS,
            observation_calls_used=self.observation_calls, observations_in_current_cycle=self.observations_in_cycle,
            active_recovery_cycle=self.recovery_cycle_active,
            allowed_actions=list(CONTRACT['actions']),
            force_interruption=self.force_stop,
            force_stop_history=list(self.force_stop_history),
            observation_history=list(self.observation_history)), images

    def _archive_navigation_progress(self):
        if self.progress is not None:
            self.previous_attempts.append(self.progress.snapshot(
                self.plant.robot_state, self.applied_controls, self.solver_calls))
            self.progress = None

    def _claim_recovery_cycle(self, result):
        if (self.instruction is not None or self.force_stop is not None) and not self.recovery_cycle_active:
            self.recoveries += 1
            self.recovery_cycle_active = True

    async def _decision_loop(self):
        while True:
            event = await self.decision_queue.get()
            if event['generation'] != self.generation:
                self.record('decision_event_discarded', event=event, reason='Motion authority superseded this queued event')
                continue
            await self._await_standing_observation(event['generation'])
            if event['generation'] != self.generation:
                self.record('decision_event_discarded', event=event, reason='Authority changed while settling')
                continue
            before = self.plant.robot_state.pose_odom.copy()
            decision_generation = self.generation
            source_frame = self.frame
            context, images = self._decision_input(event, source_frame)
            result = await asyncio.to_thread(self.planner.decide, context, images)
            validate_decision(result)
            decision_record = dict(index=len(self.decision_history)+1,
                event=event['event'], response_sensor_time_s=self.plant.stamp_s,
                proposal=result, disposition='UNASSESSED')
            self.decision_history.append(decision_record)
            if decision_generation != self.generation:
                decision_record.update(disposition='DISCARDED',
                    reason='Motion authority changed while planner request was in flight')
                self.record('planner_decision_discarded', proposal=result, event=event,
                    reason='Motion authority changed while planner request was in flight', online_evidence=context,
                    pose_before=before.tolist(), image_labels=[label for label, _ in images])
                continue
            if result['action'] == 'confirm_navigation' and (
                    event['event'] != 'NAVIGATION_TARGET_REVIEW' or self.pending_navigation is None):
                raise ValueError('confirm_navigation is valid only for the pending NAVIGATION_TARGET_REVIEW')
            if result['action'] == 'navigate' and result['target']['frame_id'] != source_frame['frame'].frame_id:
                raise ValueError('Planner target frame_id differs from its supplied current RGB')
            checks = [view_freshness(source_frame, self.plant.robot_state)]
            if result['action'] == 'confirm_navigation':
                checks.append(view_freshness(self.pending_navigation['frame'], self.plant.robot_state))
            if result['action'] not in ('needs_user', 'hold_observe') and not all(c['fresh'] for c in checks):
                decision_record.update(disposition='DISCARDED',
                    reason='Estimated view moved while the planner was thinking')
                self.record('planner_decision_discarded', proposal=result, event=event,
                    reason='Estimated view moved while the planner was thinking', freshness=checks,
                    online_evidence=context, pose_before=before.tolist(), image_labels=[label for label, _ in images])
                self._clear_pending_navigation('VIEW_CHANGED')
                self._request_planner('VIEW_CHANGED', freshness=checks, discarded_proposal=result)
                continue
            self.latest_decision = result
            decision_record['disposition'] = 'ACCEPTED'
            self.record('planner_decision', proposal=result, event=event,
                pose_before=before.tolist(), sensor_time_after=self.plant.stamp_s,
                online_evidence=context, image_labels=[label for label, _ in images], freshness=checks)
            if self.force_stop is not None:
                self.record('external_force_stop_planner_decision', proposal=result,
                    resume_authority='planner', requested_motion_resume=result['action'] in ('navigate','turn_observe','advance_observe'))
            if result['action'] in ('navigate', *SKILLS):
                self._claim_recovery_cycle(result)
            if result['action'] in SKILLS:
                self._clear_pending_navigation('PLANNER_CHOSE_OBSERVATION')
                if result['action'] != 'hold_observe':
                    self._resume_external_force(result)
                self._archive_navigation_progress()
                self.observation_calls += 1
                self.observations_in_cycle += 1
                self.path = None
                self.command[:] = 0.
                self.current_control_id = None
                self.generation += 1
                self.observation = ObservationSkill(result, self.plant.robot_state,
                    self.output/'observation_skills', self.observation_calls,
                    [self.controller.config['max_acceleration_m_s2'], self.controller.config['max_yaw_acceleration_rad_s2']])
                self._transition('OBSERVING', skill_id=self.observation_calls, proposal=result,
                    navigation_history='Retained; inference suspended during explicit observation skill')
            elif result['action'] == 'navigate':
                self._propose_navigation(result, source_frame)
            elif result['action'] == 'confirm_navigation':
                self._confirm_navigation(result)
            elif result['action'] == 'complete':
                self._clear_pending_navigation('PLANNER_DECLARED_COMPLETE')
                self.command[:] = 0.
                self.stop_requested_s = self.plant.stamp_s
                self._transition('STOPPING', planner_claim=result)
            else:
                self._clear_pending_navigation('PLANNER_REQUESTED_HELP')
                self._transition('NEEDS_USER', planner_claim=result)
                self.done.set()

    async def _navigation_loop(self):
        async with aiohttp.ClientSession(trust_env=False) as session:
            async with session.ws_connect(self.config['lightnav_url']) as ws:
                async with asyncio.TaskGroup() as connection:
                    responses = asyncio.Queue()
                    connection.create_task(self._receive_navigation(ws, responses))
                    await self._navigation_requests(ws, responses)

    async def _receive_navigation(self, ws, responses):
        # aiohttp answers protocol PING only while receive() is running. Keep one
        # reader active through planner decisions and observation actions too.
        while True:
            message = await ws.receive()
            if message.type == aiohttp.WSMsgType.TEXT:
                responses.put_nowait(json.loads(message.data))
            else:
                self.record('navigation_transport_closed', message_type=message.type.name,
                    close_code=ws.close_code, detail=str(message.data), error=str(ws.exception()))
                raise ConnectionError(f'Navigation WebSocket closed: {message.type.name}, '
                    f'code={ws.close_code}, detail={message.data!r}, error={ws.exception()!r}')

    async def _navigation_requests(self, ws, responses):
        await ws.send_json({'action': 'login', 'data': {'clientId': 'rgb-'+str(uuid.uuid4())}})
        response = await asyncio.wait_for(responses.get(), timeout=10)
        if response['action'] != 'login' or response['data']['rc'] != 0:
            raise RuntimeError(f'Navigation model login failed: {response}')
        await ws.send_json({'action': 'reset', 'data': {}})
        response = await asyncio.wait_for(responses.get(), timeout=10)
        if response['action'] != 'reset' or response['data']['rc'] != 0:
            raise RuntimeError(f'Navigation history initialization failed: {response}')
        self.nav_session = str(uuid.uuid4())
        sequence, last_stamp = 0, None
        while True:
            if self.phase not in ('NAVIGATING', 'PROBING_TARGET') or self.frame is None:
                await asyncio.sleep(.02)
                continue
            probe = self.phase == 'PROBING_TARGET'
            pending = self.pending_navigation if probe else None
            if probe and pending is None:
                raise RuntimeError('PROBING_TARGET requires a pending visual target')
            current = pending['frame'] if probe else self.frame
            stamp = current['stamp_s']
            if not probe and last_stamp is not None and stamp-last_stamp < self.config['navigation_input_period_s']-1e-8:
                self.frame_event.clear()
                await self.frame_event.wait()
                continue
            generation, instruction = self.generation, self.instruction
            jpeg = current['path'].read_bytes()
            started = time.time()
            await ws.send_json({'action': 'next', 'data': {'seq': sequence,
                'image': base64.b64encode(jpeg).decode(), 'instruction': instruction}})
            response = await asyncio.wait_for(responses.get(), timeout=self.config['navigation_response_timeout_s'])
            if response['action'] != 'next' or response['data']['rc'] != 0 or response['data']['seq'] != sequence:
                raise RuntimeError(f'Navigation inference protocol failure: {response}')
            result = response['data']
            discarded = generation != self.generation or self.phase != ('PROBING_TARGET' if probe else 'NAVIGATING')
            freshness = view_freshness(current, self.plant.robot_state) if probe else None
            stale_probe = probe and not discarded and not freshness['fresh']
            metadata = dict(sequence=sequence, session_id=self.nav_session,
                session_reset_before_input=sequence == 0, instruction=instruction,
                frame_id=current['frame'].frame_id, image=str(current['path']),
                capture_time_s=stamp, input_interval_s=None if last_stamp is None else stamp-last_stamp,
                capture_pose_estimated_odom=current['frame'].state.pose_odom.tolist(),
                request_sent_epoch=started, response_received_epoch=time.time(),
                nominal_model_input_fps=4, actual_time_prefix_modified=False,
                result=result, discarded=discarded or stale_probe,
                target_probe=probe, target_handoff_id=None if pending is None else pending['record']['handoff_id'],
                probe_view_freshness=freshness)
            self.record('navigation_model', **metadata)
            sequence += 1
            last_stamp = stamp
            if discarded:
                continue
            if stale_probe:
                self._clear_pending_navigation('VIEW_CHANGED_DURING_PROBE')
                self._request_planner('VIEW_CHANGED', freshness=[freshness])
                continue
            self.latest_model = model_observation(dict(metadata, stamp_s=self.plant.stamp_s))
            self.last_model_image = current['path']
            if probe:
                self._probe_ready(result)
            elif result['stop']:
                self._decision_event('MODEL_STOP', model=self.latest_model)
            else:
                self.path = project_path(result['actions']['actions'], current['frame'].state.pose_odom)
                self.path_stamp = stamp
                self.path_model_sequence = metadata['sequence']

    async def run(self, *, preview_only=False, preview_seconds=2.):
        self.preview = preview_only
        if not preview_only and not self.goal:
            raise ValueError('The owner must select the new task before navigation')
        workers = [asyncio.create_task(self._plant_loop(preview_seconds))]
        if not preview_only:
            workers += [asyncio.create_task(self._navigation_loop()), asyncio.create_task(self._decision_loop())]
        waiter = asyncio.create_task(self.done.wait())
        error = None
        try:
            finished, _ = await asyncio.wait([*workers, waiter], return_when=asyncio.FIRST_COMPLETED)
            for task in finished:
                task.result()
        except BaseException as failure:
            error = failure_description(failure)
            self._transition('ERROR', error=error)
            raise
        finally:
            self.command[:] = 0.
            if isinstance(self.planner, CodexPlanner):
                self.planner.close()
            for task in [*workers, waiter]:
                task.cancel()
            await asyncio.gather(*workers, waiter, return_exceptions=True)
            primary_failed = error is not None
            self.writer.stdin.close()
            video_code = self.writer.wait(timeout=15)
            self.video_log.close()
            if video_code != 0:
                self.record('video_error', exit_code=video_code)
                if error is None:
                    error = f'Video encoder exited {video_code}'
                    self._transition('ERROR', error=error)
            summary = dict(status=self.phase, error=error, goal=self.goal,
                full_navigation_tasks=0 if preview_only else 1,
                planner_declared_complete=bool(self.latest_decision and self.latest_decision['action'] == 'complete'),
                estimated_stable_stop=(self.force_stop.get('estimated_stable_stop') if self.force_stop is not None
                    else self.stable_stop if self.stop_requested_s is not None else None),
                force_stop=self.plant.force_stop.payload(), force_interruption=self.force_stop,
                force_stop_history=self.force_stop_history,
                physical_success=None,
                physical_success_note='Not evaluated by the online RGB-only pipeline',
                applied_controls=self.applied_controls, solver_calls=self.solver_calls, recoveries=self.recoveries,
                observation_calls=self.observation_calls, observation_history=self.observation_history,
                target_handoffs=self.handoff_history,
                controller=self.controller.info, navigation_session_id=self.nav_session,
                depth_used=False, global_truth_used=False, read_only_references=self.references())
            (self.output/'results.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
            self.log.close()
            self.control_log.close()
            self.execution_log.close()
            self.sensor_log.close()
            if video_code != 0 and not primary_failed:
                raise RuntimeError(f'Video encoder exited {video_code}; see {self.output/"video.log"}')
        return summary

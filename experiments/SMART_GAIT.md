PUBLIC_IP=10.13.11.197 LIVESTREAM=1 python scripts/experiments/uniFP_aliengo/smart_gait.py \
  --checkpoint logs/rsl_rl/uniFP_aliengo_position_force/2026-04-12_22-21-15_ext_force_only/model_59999.pt \
  --output-csv logs/experiments/unifp_smart_gait/human_guide_tuning.csv \
  --force-schedule human_guide \
  --stop-go-mag-tau-s 0.08 \
  --enter-score-threshold 0.50 \
  --enter-tau-on-s 0.08 \
  --enter-hold-s 0.06 \
  --exit-score-threshold 0.60 \
  --exit-tau-on-s 0.08 \
  --exit-hold-s 0.08 \
  --exit-threshold-n 0.40

#just ok 
进入 walking 平均延迟：0.77s -> 0.55s
退出 standing 平均延迟：1.10s -> 0.80s


PUBLIC_IP=10.13.11.197 LIVESTREAM=1 python scripts/experiments/uniFP_aliengo/smart_gait.py \
  --checkpoint logs/rsl_rl/uniFP_aliengo_position_force/2026-04-12_22-21-15_ext_force_only/model_59999.pt \
  --output-csv logs/experiments/unifp_smart_gait/human_guide_tuning_v2.csv \
  --force-schedule human_guide \
  --log-interval 30 \
  --stop-go-mag-tau-s 0.06 \
  --enter-score-threshold 0.46 \
  --enter-tau-on-s 0.06 \
  --enter-hold-s 0.04 \
  --exit-score-threshold 0.55 \
  --exit-tau-on-s 0.06 \
  --exit-hold-s 0.06 \
  --exit-threshold-n 0.50

#太过激进




PUBLIC_IP=10.13.11.197 LIVESTREAM=1 python scripts/experiments/uniFP_aliengo/smart_gait.py \
  --checkpoint logs/rsl_rl/uniFP_aliengo_position_force/2026-04-14_22-28-32_ext_force_only_lowerHistory/model_28000.pt \
  --policy-history-length 24 \
  --output-csv logs/experiments/unifp_smart_gait/24history_ha.csv \
  --force-schedule human_guide

#超出预期，还有些许误切换

PUBLIC_IP=10.13.11.197 LIVESTREAM=1 python scripts/experiments/uniFP_aliengo/smart_gait.py \
  --checkpoint logs/rsl_rl/uniFP_aliengo_position_force/2026-04-14_22-54-52_ext_force_only_lowActuatorHistory/model_27600.pt \
  --policy-history-length 24 \
  --output-csv logs/experiments/unifp_smart_gait/24history.csv \
  --force-schedule human_guide

#灵敏但是误切换更多


PUBLIC_IP=10.13.11.197 LIVESTREAM=1 python scripts/experiments/uniFP_aliengo/smart_gait.py \
  --checkpoint logs/rsl_rl/uniFP_aliengo_position_force/2026-04-14_22-28-32_ext_force_only_lowerHistory/model_28000.pt \
  --policy-history-length 24 \
  --output-csv logs/experiments/unifp_smart_gait/24history_ha_v2.csv \
  --force-schedule human_guide \
  --stop-go-mag-tau-s 0.08 \
  --enter-score-threshold 0.55 \
  --enter-tau-on-s 0.08 \
  --enter-hold-s 0.06 \
  --exit-score-threshold 0.60 \
  --exit-tau-on-s 0.08 \
  --exit-hold-s 0.08 \
  --exit-threshold-n 0.40 \
  --baseline-margin-n 0.20 \
  --log-interval 30

  #表现更差

  PUBLIC_IP=10.13.11.197 LIVESTREAM=1 python scripts/experiments/uniFP_aliengo/smart_gait.py \
  --checkpoint logs/rsl_rl/uniFP_aliengo_position_force/2026-04-14_22-28-32_ext_force_only_lowerHistory/model_28000.pt \
  --policy-history-length 24 \
  --output-csv logs/experiments/unifp_smart_gait/24history_ha_v3.csv \
  --force-schedule human_guide \
  --stop-go-mag-tau-s 0.08 \
  --enter-score-threshold 0.50 \
  --enter-tau-on-s 0.08 \
  --enter-hold-s 0.06 \
  --exit-score-threshold 0.60 \
  --exit-tau-on-s 0.08 \
  --exit-hold-s 0.08 \
  --exit-threshold-n 0.40 \
  --baseline-margin-n 0.15 \
  --baseline-tau-s 1.0 \
  --baseline-update-max-n 0.40 \
  --switch-cooldown-s 0.30 \
  --log-interval 30
#表现很差





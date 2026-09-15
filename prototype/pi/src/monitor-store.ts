import { appendFileSync, closeSync, mkdirSync, openSync } from "node:fs";
import { dirname } from "node:path";
import { DatabaseSync } from "node:sqlite";

export class MonitorStore {
  private readonly db: DatabaseSync;
  private readonly logFd: number;

  constructor(dbPath: string, logPath: string) {
    mkdirSync(dirname(dbPath), { recursive: true });
    mkdirSync(dirname(logPath), { recursive: true });
    this.db = new DatabaseSync(dbPath);
    this.db.exec(`
      CREATE TABLE IF NOT EXISTS events (
        seq INTEGER PRIMARY KEY AUTOINCREMENT,
        at TEXT NOT NULL,
        task_id TEXT,
        revision INTEGER,
        kind TEXT NOT NULL,
        data TEXT NOT NULL
      );
      CREATE TABLE IF NOT EXISTS tasks (
        id TEXT PRIMARY KEY,
        document TEXT NOT NULL,
        updated_at TEXT NOT NULL
      );
      CREATE TABLE IF NOT EXISTS meta (
        key TEXT PRIMARY KEY,
        value TEXT NOT NULL
      );
    `);
    this.logFd = openSync(logPath, "a");
  }

  private insertEvent(kind: string, data: unknown, taskId: string | undefined,
    revision: number | undefined, at: string): string {
    const event = { at, task_id: taskId ?? null, revision: revision ?? null, kind, data };
    const result = this.db.prepare(
      "INSERT INTO events (at, task_id, revision, kind, data) VALUES (?, ?, ?, ?, ?)",
    ).run(at, event.task_id, event.revision, kind, JSON.stringify(data));
    return `${JSON.stringify({ seq: Number(result.lastInsertRowid), ...event })}\n`;
  }

  record(kind: string, data: unknown, taskId?: string, revision?: number): void {
    const line = this.insertEvent(kind, data, taskId, revision, new Date().toISOString());
    appendFileSync(this.logFd, line);
  }

  saveTask(task: { id: string; revision: number; status: string }, kind: string, data: unknown): void {
    const at = new Date().toISOString();
    const document = JSON.stringify(task);
    let line: string;
    this.db.exec("BEGIN");
    try {
      this.db.prepare(`
        INSERT INTO tasks (id, document, updated_at) VALUES (?, ?, ?)
        ON CONFLICT(id) DO UPDATE SET document = excluded.document, updated_at = excluded.updated_at
      `).run(task.id, document, at);
      this.db.prepare(`
        INSERT INTO meta (key, value) VALUES ('latest_task_id', ?)
        ON CONFLICT(key) DO UPDATE SET value = excluded.value
      `).run(task.id);
      line = this.insertEvent(kind, data, task.id, task.revision, at);
      this.db.exec("COMMIT");
    } catch (error) {
      this.db.exec("ROLLBACK");
      throw error;
    }
    // SQLite owns the durable transaction; the append-only JSONL mirror follows commit.
    appendFileSync(this.logFd, line);
  }

  loadLatest(): unknown | undefined {
    const row = this.db.prepare(`
      SELECT document FROM tasks WHERE id = (SELECT value FROM meta WHERE key = 'latest_task_id')
    `).get();
    return row === undefined ? undefined : JSON.parse(row.document as string);
  }

  recallSegments(taskId: string, stageIndex: number, failedAttemptsOnly: boolean): unknown[] {
    const rows = this.db.prepare(`
      SELECT data FROM events
      WHERE task_id=? AND kind='decision_segment_archived'
        AND json_extract(data, '$.stageIndex')=?
      ORDER BY seq DESC LIMIT 12
    `).all(taskId, stageIndex) as { data: string }[];
    const segments = rows.map(row => JSON.parse(row.data));
    return (failedAttemptsOnly
      ? segments.filter(segment => segment.motion?.events.some((event: { reason: string }) =>
        ["mpc_blocked", "no_progress"].includes(event.reason)))
      : segments).slice(0, 2);
  }

  eventCount(): number {
    const row = this.db.prepare("SELECT COUNT(*) AS count FROM events").get() as { count: number };
    return row.count;
  }

  close(): void {
    this.db.close();
    closeSync(this.logFd);
  }
}

import { ipcMain, WebContents } from 'electron';
import { ChildProcess, spawn } from 'child_process';
import path from 'path';
import readline from 'readline';
import channels from 'common/channels';

export interface EngineEvent {
  kind: 'status' | 'log' | 'progress' | 'done';
  text?: string;
  value?: number;
  ok?: boolean;
}

const running = new Map<number, ChildProcess>();

export class XPlaneEngine {
  static executable(): string {
    const dir = import.meta.env.PROD ? path.join(process.resourcesPath, 'extraResources') : 'extraResources';
    return path.resolve(dir, 'a380x-xp-engine.exe');
  }

  static query(args: string[]): Promise<unknown> {
    return new Promise((resolve, reject) => {
      const child = spawn(XPlaneEngine.executable(), args, { windowsHide: true });
      let out = '';
      let err = '';
      child.stdout.on('data', (d) => (out += d));
      child.stderr.on('data', (d) => (err += d));
      child.on('error', reject);
      child.on('close', (code) => {
        try {
          resolve(JSON.parse(out));
        } catch {
          reject(new Error(err.trim() || `The installer engine exited with code ${code}`));
        }
      });
    });
  }

  static install(sender: WebContents, installID: number, args: string[]): Promise<EngineEvent> {
    return new Promise((resolve) => {
      const child = spawn(XPlaneEngine.executable(), args, { windowsHide: true });
      running.set(installID, child);

      let result: EngineEvent | null = null;
      let stderr = '';
      child.stderr.on('data', (d) => (stderr += d));

      readline.createInterface({ input: child.stdout }).on('line', (line) => {
        let event: EngineEvent;
        try {
          event = JSON.parse(line);
        } catch {
          event = { kind: 'log', text: line };
        }
        if (event.kind === 'done') {
          result = event;
        }
        sender.send(channels.xplaneEngine.event, installID, event);
      });

      child.on('error', (e) => {
        result = { kind: 'done', ok: false, text: e.message };
      });

      child.on('close', (code, signal) => {
        running.delete(installID);
        if (signal) {
          resolve({ kind: 'done', ok: false, text: 'cancelled' });
        } else {
          resolve(
            result ?? {
              kind: 'done',
              ok: false,
              text: stderr.trim() || `The installer engine exited with code ${code}`,
            },
          );
        }
      });
    });
  }

  static cancel(installID: number): void {
    running.get(installID)?.kill();
  }

  static setupIpcListeners(): void {
    ipcMain.handle(channels.xplaneEngine.query, (_, args: string[]) => XPlaneEngine.query(args));
    ipcMain.handle(channels.xplaneEngine.install, (event, installID: number, args: string[]) =>
      XPlaneEngine.install(event.sender, installID, args),
    );
    ipcMain.on(channels.xplaneEngine.cancel, (_, installID: number) => XPlaneEngine.cancel(installID));
  }
}

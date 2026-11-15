import fs from 'node:fs';
import path from 'node:path';
import { execSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const AGENTS_DIR = path.resolve(__dirname, '..');
const CHECKPOINT_PATH = path.join(AGENTS_DIR, 'session_checkpoint.json');
const HANDOFF_PATH = path.join(AGENTS_DIR, 'SESSION_HANDOFF.md');

function readStdin() {
  return new Promise((resolve) => {
    let data = '';
    process.stdin.setEncoding('utf8');
    process.stdin.on('data', (chunk) => {
      data += chunk;
    });
    process.stdin.on('end', () => {
      resolve(data);
    });
  });
}

function truncate(str, maxLen) {
  if (!str || typeof str !== 'string') return '';
  const cleaned = str.trim();
  if (cleaned.length <= maxLen) return cleaned;
  return cleaned.slice(0, maxLen) + '... (truncated)';
}

function getGitSummary(cwd) {
  const targetDir = cwd && fs.existsSync(cwd) ? cwd : path.resolve(AGENTS_DIR, '..');
  try {
    const branch = execSync('git branch --show-current', {
      cwd: targetDir,
      timeout: 2000,
      stdio: ['ignore', 'pipe', 'ignore'],
      encoding: 'utf8',
    }).trim();
    const status = execSync('git status --short', {
      cwd: targetDir,
      timeout: 2000,
      stdio: ['ignore', 'pipe', 'ignore'],
      encoding: 'utf8',
    }).trim();
    const statusLines = status ? status.split(/\r?\n/).slice(0, 15) : [];
    return {
      branch: branch || '(detached)',
      dirtyCount: status ? status.split(/\r?\n/).length : 0,
      statusPreview: statusLines.join('\n'),
    };
  } catch {
    return null;
  }
}

function extractTranscriptSummary(transcriptPath) {
  if (!transcriptPath || !fs.existsSync(transcriptPath)) {
    return { lastUser: '', lastAssistant: '' };
  }
  try {
    const stat = fs.statSync(transcriptPath);
    const maxBytes = 128 * 1024;
    const start = Math.max(0, stat.size - maxBytes);
    const fd = fs.openSync(transcriptPath, 'r');
    const buffer = Buffer.alloc(stat.size - start);
    fs.readSync(fd, buffer, 0, buffer.length, start);
    fs.closeSync(fd);

    const text = buffer.toString('utf8');
    const lines = text.split(/\r?\n/);
    if (start > 0 && lines.length > 0) {
      lines.shift(); // drop partial first line
    }

    let lastUser = '';
    let lastAssistant = '';
    for (const line of lines) {
      if (!line.trim()) continue;
      try {
        const entry = JSON.parse(line);
        if (entry.type === 'USER_INPUT' && entry.content) {
          lastUser = entry.content;
        } else if (entry.type === 'PLANNER_RESPONSE' && entry.content) {
          lastAssistant = entry.content;
        }
      } catch {
        // ignore malformed line
      }
    }
    return {
      lastUser: truncate(lastUser, 500),
      lastAssistant: truncate(lastAssistant, 800),
    };
  } catch {
    return { lastUser: '', lastAssistant: '' };
  }
}

function handleStop(payload) {
  const transcriptSummary = extractTranscriptSummary(payload.transcriptPath);
  const git = getGitSummary(payload.cwd);

  const checkpoint = {
    savedAt: new Date().toISOString(),
    conversationId: payload.conversationId || '',
    transcriptPath: payload.transcriptPath || '',
    artifactDirectoryPath: payload.artifactDirectoryPath || '',
    modelName: payload.modelName || '',
    cwd: payload.cwd || '',
    terminationReason: payload.terminationReason || '',
    error: payload.error || '',
    lastUserInput: truncate(payload.lastUserInput || transcriptSummary.lastUser, 500),
    lastAssistantSummary: transcriptSummary.lastAssistant,
    git,
  };

  fs.mkdirSync(AGENTS_DIR, { recursive: true });
  fs.writeFileSync(CHECKPOINT_PATH, JSON.stringify(checkpoint, null, 2), 'utf8');
  return {};
}

function handleSessionStart(payload) {
  const hasCheckpoint = fs.existsSync(CHECKPOINT_PATH);
  const hasHandoff = fs.existsSync(HANDOFF_PATH);

  if (!hasCheckpoint && !hasHandoff) {
    return {};
  }

  const parts = [];
  let checkpoint = null;

  if (hasCheckpoint) {
    try {
      checkpoint = JSON.parse(fs.readFileSync(CHECKPOINT_PATH, 'utf8'));
    } catch {
      checkpoint = null;
    }
  }

  if (checkpoint && checkpoint.conversationId !== payload.conversationId) {
    parts.push('## Previous Session Checkpoint');
    if (checkpoint.savedAt) parts.push(`- **Saved At**: ${checkpoint.savedAt}`);
    if (checkpoint.conversationId) parts.push(`- **Previous Conversation ID**: ${checkpoint.conversationId}`);
    if (checkpoint.terminationReason) parts.push(`- **Termination Reason**: ${checkpoint.terminationReason}`);
    if (checkpoint.error) parts.push(`- **Error**: ${checkpoint.error}`);
    if (checkpoint.transcriptPath) parts.push(`- **Previous Transcript**: ${checkpoint.transcriptPath}`);
    if (checkpoint.artifactDirectoryPath) parts.push(`- **Previous Artifacts**: ${checkpoint.artifactDirectoryPath}`);
    if (checkpoint.lastUserInput) parts.push(`- **Last User Request**: ${checkpoint.lastUserInput}`);
    if (checkpoint.lastAssistantSummary) {
      parts.push(`- **Last Assistant Response**:\n${checkpoint.lastAssistantSummary}`);
    }
    if (checkpoint.git) {
      parts.push(`- **Git Branch (at stop)**: ${checkpoint.git.branch} (${checkpoint.git.dirtyCount} changed)`);
    }
  }

  if (hasHandoff) {
    try {
      const handoffText = truncate(fs.readFileSync(HANDOFF_PATH, 'utf8'), 4000);
      if (handoffText) {
        parts.push('## Session Handoff Notes (.agents/SESSION_HANDOFF.md)\n' + handoffText);
      }
    } catch {
      // ignore
    }
  }

  if (parts.length === 0) {
    return {};
  }

  parts.push(
    'If the user asks to continue or resume previous work, use the context above (and read the previous transcript if full details are needed) without asking the user to repeat everything.'
  );

  return {
    injectSteps: [
      {
        systemMessage: {
          systemMessage: parts.join('\n\n'),
        },
      },
    ],
  };
}

async function main() {
  const raw = await readStdin();
  let payload = {};
  try {
    payload = raw.trim() ? JSON.parse(raw) : {};
  } catch (err) {
    process.stderr.write(`session_continuity parse error: ${err?.message || err}\n`);
    process.stdout.write('{}');
    return;
  }

  const event = payload.hookEventName || '';
  let result = {};
  if (event === 'Stop') {
    result = handleStop(payload);
  } else if (event === 'SessionStart') {
    result = handleSessionStart(payload);
  }

  process.stdout.write(JSON.stringify(result));
}

main().catch((err) => {
  process.stderr.write(`session_continuity error: ${err?.message || err}\n`);
  process.stdout.write('{}');
});

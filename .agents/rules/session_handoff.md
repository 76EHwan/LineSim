# Session Continuity & Handoff Rule

To ensure work can be resumed seamlessly if a session disconnects or hits context limits:

1. **Maintain Handoff Notes on Multi-Step Tasks**:
   - When working on a non-trivial or multi-step task, update `.agents/SESSION_HANDOFF.md` at major milestones and before wrapping up.
   - Keep it concise and structured:
     - **Current Goal**: What is being solved.
     - **Completed**: Key changes made, files modified, and verified results.
     - **Next Steps**: Remaining tasks or open issues.
     - **Key Files & Commands**: Relevant paths and commands to resume immediately.

2. **Resuming a Previous Session**:
   - At session start, `.agents/hooks/session_continuity.mjs` automatically injects the previous session's checkpoint (`session_checkpoint.json`) and `SESSION_HANDOFF.md` if available.
   - When the user asks to continue previous work (e.g., "이어서 해줘", "계속 진행해", "resume"), review the injected context and read the previous `transcriptPath` if detailed history is needed before proceeding.

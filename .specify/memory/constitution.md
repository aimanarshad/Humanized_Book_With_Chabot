Title: Physical AI & Humanoid Robotics — Spec-Driven Book Constitution

Mission:
- To build a fully automated RAG-based chatbot for the Physical AI book using Claude Code, Spec-Kit Plus, OpenAI Agents/ChatKit SDK, FastAPI backend, Qdrant Cloud vector store, Neon Serverless Postgres, and Docusaurus frontend.
- Produce a modular, spec-driven educational book that teaches Physical AI and Humanoid Robotics from simulation to deployment.
- Output: a Docusaurus site (Markdown docs/) deployed to GitHub Pages with reproducible examples, notebooks, and code.

Scope:
- Core modules: Intro, Module1[ROS2], Module2[Gazebo+Unity], Module3[NVIDIA Isaac], Module4[VLA], Capstone, Appendices (Hardware + Software).
- Audience: advanced undergraduates and engineers with programming and basic robotics background.
- Deliverables: Markdown chapters, working example code, CI to build and publish, Docker/MCP wrappers for reproducible simulation, and unit / integration tests.

Principles & Rules:
1. Spec-first: every chapter and code artifact MUST have a Spec-Kit spec before implementation.
2. Reproducibility: examples must run with documented OS & container images (Ubuntu 22.04 base).
3. Minimal secrets: store no credentials in repo; use GitHub Actions secrets for deployments.
4. Modularity: each module is independent (can be read/tested individually).
5. Progressive complexity: start simple (publish quick runnable examples) then add advanced sections (Isaac, RL).
6. Licensing: MIT for code examples; Creative Commons Attribution (CC-BY-SA) for text content.
7. Accessibility: include alt-text for diagrams and runnable notebooks where possible.

Quality Gates (must pass to publish a chapter):
- Spec exists (/sp.specify) and is clarified (/sp.clarify).
- Unit tests or smoke tests for examples.
- Docusaurus preview builds locally (npm run start) and passes CI build job.
- ADR created for any non-trivial architectural decision.

Release cadence:
- Alpha (module drafts) every 2 weeks.
- Beta (complete module + tests) monthly.
- Final book release when all modules and capstone pass CI and ADRs reviewed.

Ownership & Roles:
- Editor: coordinates specs and final edits.
- Module Leads: own module specs, plans and implementations.
- CI / DevOps: manage Docusaurus site and GH Pages deploy.

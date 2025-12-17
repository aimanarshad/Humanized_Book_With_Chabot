# Feature Tasks: Capstone Project

**Feature Branch**: `001-capstone-project` | **Date**: 2025-12-06 | **Spec**: specs/001-capstone-project/spec.md
**Plan**: specs/001-capstone-project/plan.md

## Phase 1: Setup (Project Initialization)

- [ ] T001 Create physical-ai-book repository skeleton: physical-ai-book/ with docs/, sidebars.js, docusaurus.config.js, package.json

## Phase 2: Foundational (Blocking Prerequisites)

- [ ] T002 Write Introduction to Physical AI outline and learning outcomes: docs/intro.md
- [ ] T003 Validate Module 1 (ROS2) specification: specs/001-module1-ros2/spec.md
- [ ] T004 Implement simple rclpy publisher/subscriber with smoke test script: examples/module1_ros2/talker_listener/
- [ ] T005 Create simplified URDF and ros2 launch file for humanoid: examples/module1_ros2/urdf/
- [ ] T006 Implement SDF/URDF and launch for minimal Gazebo robot, including depth camera sensor plugins: examples/gazebo_sim/
- [ ] T007 Implement ROS 2 node that subscribes to simulated sensors and logs data: examples/gazebo_sim/sensor_logger_node.py
- [ ] T008 Script to generate Isaac sample dataset: examples/isaac_pipeline/dataset_generator.py

## Phase 3: User Story 1 - End-to-End Voice Command to Humanoid Action (Priority: P1)

Goal: Users want to see an end-to-end simulated humanoid responding to voice commands, perceiving its environment, planning its actions, and executing them.
Independent Test: Can be fully tested by providing a voice command and observing the simulated humanoid successfully completing the corresponding task in the simulation.

- [ ] T009 [P] [US1] Implement ROS 2 voice command node: examples/capstone/voice_command_node.py
- [ ] T010 [P] [US1] Implement ROS 2 perception node: examples/capstone/perception_node.py
- [ ] T011 [P] [US1] Implement LLM planner ROS 2 node: examples/capstone/llm_planner_node.py
- [ ] T012 [P] [US1] Implement ROS 2 action server node: examples/capstone/action_server_node.py
- [ ] T013 [US1] Create simulation launch file for Capstone: examples/capstone/simulation_launch.launch
- [ ] T014 [US1] Integrate voice command, perception, LLM planning, and action server nodes for end-to-end demo

## Phase 4: User Story 2 - Comprehensive Capstone Guide (Priority: P1)

Goal: Users want a step-by-step guide to understand and reproduce the Capstone Project.
Independent Test: Can be fully tested by following the guide and successfully setting up and running the Capstone Project.

- [ ] T015 [US2] Write step-by-step Capstone guide: docs/capstone.md

## Phase 5: User Story 3 - Automated Validation with CI Smoke Test (Priority: P2)

Goal: Users want to ensure the Capstone Project's functionality is continuously validated through automated tests.
Independent Test: Can be fully tested by running the CI smoke test and verifying that it passes without errors.

- [ ] T016 [US3] Implement CI smoke test for Capstone project: .github/workflows/capstone_ci.yml

## Phase 6: Polish & Cross-Cutting Concerns

- [ ] T017 Verify all previous modules are referenced and integrated in Capstone documentation and code (Corresponds to User Story 4 - Integration of All Modules, P1)

## Dependencies

- T002 depends on T001
- T003 depends on T002
- T004 depends on T003
- T005 depends on T004
- T006 depends on T005
- T007 depends on T006
- T008 depends on T007
- T009 depends on T008 (assumes foundational setup complete)
- T010 depends on T008 (can be parallel with T009)
- T011 depends on T008 (can be parallel with T009, T010)
- T012 depends on T008 (can be parallel with T009, T010, T011)
- T013 depends on T009, T010, T011, T012
- T014 depends on T013
- T015 depends on T014 (documentation can be written as integration progresses)
- T016 depends on T014 (CI can be set up after core demo is functional)
- T017 depends on T014, T015, T016 (final verification)

## Parallel Execution Examples

### For User Story 1 (End-to-End Voice Command to Humanoid Action):

- **Parallel tasks for node implementation (after foundational setup):**
  - Implement ROS 2 voice command node: examples/capstone/voice_command_node.py
  - Implement ROS 2 perception node: examples/capstone/perception_node.py
  - Implement LLM planner ROS 2 node: examples/capstone/llm_planner_node.py
  - Implement ROS 2 action server node: examples/capstone/action_server_node.py

## Implementation Strategy

The implementation will follow an MVP-first approach, focusing on completing User Story 1 (End-to-End Voice Command to Humanoid Action) as the initial demonstrable increment. Subsequent user stories will be addressed in priority order, with continuous integration and documentation updates throughout the process. Dependencies between tasks will be managed to ensure a smooth workflow and maximize parallel development opportunities where possible.

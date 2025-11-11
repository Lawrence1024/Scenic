# Scenic–dSPACE Documentation Hub

This directory is the living knowledge base for the Scenic ↔︎ dSPACE integration. It consolidates architecture, control, runtime, and operations guidance for engineers and AI assistants.

> Tip: Start with the two overview guides below, then dive into Control, Runtime, or Deep‑Dive sections as needed. Where guidance overlaps, prefer the “2025‑11 Updates” sections for the latest behavior.

---

## 📚 Overview & Architecture

### 🏗️ Core Architecture
- **[SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md](./SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md)**  
  Comprehensive description of Scenic’s domain model and patterns:
  - Domain layering (core → driving → racing → simulators)
  - Protocols (`Steers`, `HasManualTransmission`), actions, and behaviors
  - “Extend, don’t replace” design

### 🧰 dSPACE Integration (Single Source of Truth)
- **[DSPACE_COMPREHENSIVE_GUIDE.md](./DSPACE_COMPREHENSIVE_GUIDE.md)**  
  End‑to‑end reference for the dSPACE simulator integration:
  - Simulator structure & components (ModelDesk/ControlDesk/VEOS)
  - Vehicle control module (ego via VesiInterface, fellow via External Signals)
  - XODR → RD coordinate transform & geometry pipeline
  - ModelDesk authoring flow & ControlDesk initialization
  - Troubleshooting & ops checklist
  - ▶︎ 2025‑11 Updates: warm‑up gating, bulk ExternalSignals write/readback, segment “Continue”, reduced startup noise
  - Supersedes: DSPACE_SIMULATOR_STRUCTURE.md, DSPACE_CONTROL_INTERFACES.md, DSPACE_COORDINATE_TRANSFORMATION.md

---

## 🎮 Control & Actions

- **[VEHICLE_CONTROL_IMPLEMENTATION.md](./VEHICLE_CONTROL_IMPLEMENTATION.md)**  
  Domain‑level control and dSPACE mapping:
  - `Steers` (throttle/steer/brake) & `HasManualTransmission` (gear/clutch)
  - Action accumulation & one‑shot vs continuous controls
  - VesiInterface and External Signals paths & unit conversions
  - ▶︎ 2025‑11 Updates: bulk ExternalSignals write/readback, index/unit probing, warm‑up gating, seg1 “Continue”

- **[CONTROLDESK_JOYSTICK_INTEGRATION.md](./CONTROLDESK_JOYSTICK_INTEGRATION.md)**  
  Configure ControlDesk to drive the car manually; tips for axes, scaling, and calibration.

---

## 🔧 Runtime & Data Flow

- **[SIMULATION_LOOP_ \_FLOW.md](./SIMULATION_LOOP_FLOW.md)**  
  How `executeActions → step → getProperties` orchestrates control and sensing:
  - dSPACE `SingleStep` timing & Online Calibration
  - `dspaceActor` state flow (position/velocity/yaw)
  - ▶︎ 2025‑11 Updates: behavior deferral until plant arrays publish non‑zero data; ExternalSignals path probe; bulk reads for fellow pose

---

## 🤿 Deep Dives & Change Notes

- **[STEP_AND_GETPROPERTIES_FIX.md](./STEP_AND_GETPROPERTIES_FIX.md)** (historical deep dive)  
  Rationale and implementation of `step()`/`getProperties()` refactor using `DSpaceVehicleActor` (superseded by the runtime guide but kept for reference).

- **[DSPACEACTOR_REFACTORING.md](./DSPACEACTOR_REFACTORING.md)**  
  Design notes and refactoring details for consolidating internal state into `DSpaceVehicleActor`.

- **[DECISION_TREE_IMPLEMENTATION_ \_SUMMARY.md](./DECISION_TREE_IMPLEMENTATION_SUMMARY.md)**  
  Overview of racing decision logic (e.g., follow mode, TTL selection) and how it integrates with `RacingSteers` actions.

---

## ✅ Quick Start

- New to this repo? → Read **DSPACE_COMPREHENSIVE_GUIDE** and **SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE**
- Implementing or debugging controls? → **VEHICLE_CONTROL_IMPLEMENTATION** (see 2025‑11 updates)
- Tracing the timestep and data flow? → **SIMULATION_LOOP_FLOW**
- Using ControlDesk joystick? → **CONTROLDESK_JOYSTICK_INTEGRATION**

---

## 🔄 Maintenance

Keep these docs in sync with code changes:
- dSPACE runtime, External Signals, ModelDesk/ControlDesk → update **DSPACE_COMPREHENSIVE_GUIDE** and **SIMULATION_LOOP_FLOW**
- Protocols/actions & control mapping → update **VEHICLE_CONTROL_IMPLEMENTATION**
- Model & domain architecture → update **SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE**
- Joystick tooling → update **CONTROLDESK_JOYSTICK_INTEGRATION**

---

## 🤝 Contributing & Questions

- Use clear headings, include code paths (e.g., `scenic/simulators/dspace/simulator.py: _ensureFellowArraysInitialized`).
- Cross‑link related sections across documents.
- Open a PR or issue for gaps/updates. For quick answers, search within this folder first.

# Scenic-dSPACE AI Documentation

This directory contains comprehensive technical documentation for the Scenic-dSPACE simulator integration, created to aid AI assistants and developers in understanding the codebase.

## Document Index

### 🏗️ Architecture & Structure

- **[SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md](./SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md)** (33KB, 1020 lines)
  - Complete overview of Scenic domain architecture
  - Racing domain structure and protocols
  - Abstract vs concrete implementations
  - Best practices for domain design

- **[DSPACE_SIMULATOR_STRUCTURE.md](./DSPACE_SIMULATOR_STRUCTURE.md)** (16KB, 538 lines)
  - dSPACE simulator class hierarchy
  - COM automation interfaces
  - ModelDesk integration patterns
  - Simulation lifecycle and state management

### 🎮 Control & Actions

- **[VEHICLE_CONTROL_IMPLEMENTATION.md](./VEHICLE_CONTROL_IMPLEMENTATION.md)** (15KB, 447 lines)
  - Control protocol implementations
  - Steers protocol (throttle, steering, braking)
  - Manual transmission protocol (gear, clutch)
  - Action storage and application

- **[DSPACE_CONTROL_INTERFACES.md](./DSPACE_CONTROL_INTERFACES.md)** (12KB, 245 lines)
  - dSPACE COM automation API
  - Control input/output interfaces
  - Real-time control mechanisms
  - Synchronization patterns

### 📍 Coordinate Systems

- **[DSPACE_COORDINATE_TRANSFORMATION.md](./DSPACE_COORDINATE_TRANSFORMATION.md)** (20KB, 695 lines) ⭐ **NEW**
  - Complete coordinate transformation pipeline
  - XODR → RD coordinate system conversion
  - Geometric projection to (s,t) road coordinates
  - Orientation conversion (Scenic → dSPACE)
  - Calibration parameters and troubleshooting
  - **Essential for debugging positioning issues**

## Quick Reference

### For Debugging Vehicle Positioning
→ Start with **[DSPACE_COORDINATE_TRANSFORMATION.md](./DSPACE_COORDINATE_TRANSFORMATION.md)**

### For Understanding Domain Architecture
→ Read **[SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md](./SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md)**

### For Implementing Vehicle Control
→ Consult **[VEHICLE_CONTROL_IMPLEMENTATION.md](./VEHICLE_CONTROL_IMPLEMENTATION.md)**

### For dSPACE Simulator Integration
→ Study **[DSPACE_SIMULATOR_STRUCTURE.md](./DSPACE_SIMULATOR_STRUCTURE.md)**

### For COM Automation Details
→ Review **[DSPACE_CONTROL_INTERFACES.md](./DSPACE_CONTROL_INTERFACES.md)**

## Document Relationships

```
SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md
  ├─ Overview of domain abstraction
  ├─ Protocol definitions
  └─ Design patterns

DSPACE_SIMULATOR_STRUCTURE.md
  ├─ Simulator implementation
  ├─ Inherits from architecture patterns
  └─ COM integration details

VEHICLE_CONTROL_IMPLEMENTATION.md
  ├─ Control protocol implementations
  ├─ Extends domain protocols
  └─ Action storage/applying

DSPACE_CONTROL_INTERFACES.md
  ├─ COM automation specifics
  ├─ Used by simulator structure
  └─ Real-time control mechanisms

DSPACE_COORDINATE_TRANSFORMATION.md
  ├─ Independent transformation pipeline
  ├─ Used by simulator for positioning
  └─ Bridges Scenic ↔ dSPACE coordinates
```

## Key Topics Covered

### Coordinate Systems
- Scenic world coordinates → ModelDesk (s,t)
- XODR and RD coordinate transformations
- Geometric projection algorithms
- Orientation conversions
- Calibration parameters

### Protocols & Interfaces
- Abstract racing protocols
- Steers protocol implementation
- Manual transmission protocol
- dSPACE COM automation
- Action application lifecycle

### Simulator Integration
- COM automation setup
- Object creation workflow
- Control state management
- Route detection and assignment
- Track segment identification

### Architecture Patterns
- Domain abstraction layers
- Protocol-based design
- Simulator-specific implementations
- Separation of concerns

## Maintenance

These documents are maintained in sync with the codebase. When significant changes are made to:
- Coordinate transformation logic → Update DSPACE_COORDINATE_TRANSFORMATION.md
- Control protocols → Update VEHICLE_CONTROL_IMPLEMENTATION.md
- Simulator structure → Update DSPACE_SIMULATOR_STRUCTURE.md
- Architecture patterns → Update SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md

## Contributing

When adding new documentation:
1. Use clear, hierarchical markdown structure
2. Include code locations (file:line references)
3. Add examples and diagrams where helpful
4. Cross-reference related documents
5. Update this README index

## Questions or Issues

For clarification on any document, search the document first, then check referenced code locations. These documents are designed to be comprehensive references for AI assistants and developers working with the Scenic-dSPACE codebase.


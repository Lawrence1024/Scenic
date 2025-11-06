# Scenic-dSPACE AI Documentation

This directory contains comprehensive technical documentation for the Scenic-dSPACE simulator integration, created to aid AI assistants and developers in understanding the codebase.

## Document Index

### 🏗️ Architecture & Structure

- **[SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md](./SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md)** (33KB, 1020 lines)
  - Complete overview of Scenic domain architecture
  - Racing domain structure and protocols
  - Abstract vs concrete implementations
  - Best practices for domain design

- **[DSPACE_COMPREHENSIVE_GUIDE.md](./DSPACE_COMPREHENSIVE_GUIDE.md)** ⭐ **NEW - CONSOLIDATED**
  - Complete dSPACE simulator documentation in one place
  - Directory structure and core components
  - Vehicle control module (physics & controller)
  - Control interfaces (VesiInterface, ExternalUserData)
  - Coordinate transformation pipeline (XODR→RD)
  - Integration points and configuration
  - Comprehensive troubleshooting guide
  - **Replaces**: DSPACE_SIMULATOR_STRUCTURE.md, DSPACE_CONTROL_INTERFACES.md, DSPACE_COORDINATE_TRANSFORMATION.md

### 🎮 Control & Actions

- **[VEHICLE_CONTROL_IMPLEMENTATION.md](./VEHICLE_CONTROL_IMPLEMENTATION.md)** (15KB, 447 lines)
  - Control protocol implementations
  - Steers protocol (throttle, steering, braking)
  - Manual transmission protocol (gear, clutch)
  - Action storage and application

- **[CONTROLDESK_JOYSTICK_INTEGRATION.md](./CONTROLDESK_JOYSTICK_INTEGRATION.md)**
  - ControlDesk instrument script for joystick input
  - Mapping raw joystick values to ControlDesk variables
  - Steering, throttle, and brake axis handling
  - Real-time manual control integration
  - Calibration and troubleshooting guide

## Quick Reference

### For ALL dSPACE Topics
→ **[DSPACE_COMPREHENSIVE_GUIDE.md](./DSPACE_COMPREHENSIVE_GUIDE.md)** - Single comprehensive reference

### For Understanding Domain Architecture
→ Read **[SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md](./SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md)**

### For Implementing Vehicle Control
→ Consult **[VEHICLE_CONTROL_IMPLEMENTATION.md](./VEHICLE_CONTROL_IMPLEMENTATION.md)**

### For Joystick Integration
→ Consult **[CONTROLDESK_JOYSTICK_INTEGRATION.md](./CONTROLDESK_JOYSTICK_INTEGRATION.md)**

## Document Relationships

```
SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md
  ├─ Overview of domain abstraction
  ├─ Protocol definitions
  └─ Design patterns

DSPACE_COMPREHENSIVE_GUIDE.md ⭐ CONSOLIDATED
  ├─ Complete dSPACE reference (all topics)
  ├─ Simulator structure & components
  ├─ Vehicle control module (physics & controller)
  ├─ Control interfaces (VesiInterface, ExternalUserData)
  ├─ Coordinate transformation pipeline (XODR→RD)
  ├─ Integration & configuration
  └─ Troubleshooting guide

VEHICLE_CONTROL_IMPLEMENTATION.md
  ├─ Control protocol implementations
  ├─ Extends domain protocols
  └─ Action storage/applying

CONTROLDESK_JOYSTICK_INTEGRATION.md
  ├─ Joystick to ControlDesk mapping
  ├─ Instrument script implementation
  └─ Real-time manual control
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
- Joystick integration via ControlDesk instruments

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
- **ANY dSPACE topics** → Update DSPACE_COMPREHENSIVE_GUIDE.md (single source of truth)
- Control protocols → Update VEHICLE_CONTROL_IMPLEMENTATION.md
- Architecture patterns → Update SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md
- Joystick integration → Update CONTROLDESK_JOYSTICK_INTEGRATION.md

## Contributing

When adding new documentation:
1. Use clear, hierarchical markdown structure
2. Include code locations (file:line references)
3. Add examples and diagrams where helpful
4. Cross-reference related documents
5. Update this README index

## Questions or Issues

For clarification on any document, search the document first, then check referenced code locations. These documents are designed to be comprehensive references for AI assistants and developers working with the Scenic-dSPACE codebase.


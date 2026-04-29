# AI Disclosure

> **Project:** DOF-5 Cornea Focus System  
> **Author:** Anton Bloch (antbloch@uw.edu)  
> **Date:** April 2026

---

## Purpose

This document describes how AI tools were used during development of this project.

## How AI Was Used

### Documentation (README, setup guides)

The hardware initialization guide in `README.md` was written and validated by the author, with diagrams created by Claude. synthesized from Dover Motion's official product documentation (User Guide, Software Guide, Quick Start Guide). AI (GitHub Copilot) was used to verify technical accuracy and assist with formatting.

### Code & Software Architecture

All source code, system architecture, and feature design in `cornea_focus/` and `scripts/` was implemented by hand. Design decisions (pipeline structure, control loop, surface detection approach) were made by the author based on domain knowledge and project requirements.

### Engineering Best Practices

AI was consulted to verify choices around project structure, configuration patterns, and Python packaging conventions. These were treated as a reference.

### Debugging & Problem-Solving

AI was used as a diagnostic tool to work through blocking technical issues (environment configuration, network problems, driver communication) that would have otherwise delayed productivity.

AI was not used for the following:
- Generating entire source code or algorithms
- Writing research content or analysis
- Making design decisions without author review
- Producing any output that was accepted without verification

## Tools

- **GitHub Copilot** (Claude 4.7 Opus) — conversational debugging, documentation review, best-practice verification

---

*This disclosure follows emerging best practices for AI transparency in academic and research software projects.*

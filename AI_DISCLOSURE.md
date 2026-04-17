# AI Disclosure

> **Project:** DOF-5 Cornea Focus System  
> **Author:** Anton Bloch (antbloch@uw.edu)  
> **Date:** April 2026

---

## Purpose

This document describes how AI tools were used during development of this project, in the interest of academic transparency.

## How AI Was Used

### Documentation (README, setup guides)

The hardware initialization guide in `README.md` was **written by the author**, synthesized from Dover Motion's official product documentation (User Guide, Software Guide, Quick Start Guide). AI (GitHub Copilot) was used to **verify technical accuracy** and assist with formatting — not to generate the content.

### Code & Software Architecture

All source code, system architecture, and feature design in `cornea_focus/` and `scripts/` was **implemented by hand**. Design decisions (pipeline structure, control loop, surface detection approach) were made by the author based on domain knowledge and project requirements.

### Engineering Best Practices

AI was consulted to **verify** choices around project structure, configuration patterns, and Python packaging conventions. These were treated as a reference — similar to consulting documentation or Stack Overflow — not as a source of implementation.

### Debugging & Problem-Solving

AI was used as a **diagnostic tool** to work through blocking technical issues (e.g., environment configuration, network problems, driver communication) that would have otherwise delayed productivity. The author identified the problems; AI assisted in isolating root causes and solutions.

## What AI Was NOT Used For

- Generating source code or algorithms
- Writing research content or analysis
- Making design decisions without author review
- Producing any output that was accepted without verification

## Tools

- **GitHub Copilot** (Claude) — conversational debugging, documentation review, best-practice verification

---

*This disclosure follows emerging best practices for AI transparency in academic and research software projects.*

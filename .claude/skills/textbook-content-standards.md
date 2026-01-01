---
name: textbook-content-standards
description: Ensures textbook content follows Physical AI curriculum quality and technical accuracy standards. Use when writing or reviewing documentation chapters.
allowed-tools: Read, Grep, Glob
---

# Textbook Content Standards

All documentation must adhere to the quality standards defined in the project Constitution (Principle III).

## 1. Technical Accuracy
- **Authoritative Sources**: Verify technical claims against official documentation (ROS 2, NVIDIA Isaac, etc.).
- **Recency**: Hardware specs must be ≤ 90 days old.
- **Formulas**: Use KaTeX/LaTeX for all math. Ensure physics simulations are validated.

## 2. Formatting & Structure
- **MDX**: Use `.md` or `.mdx` files.
- **Frontmatter**: Every file must contain:
  ```yaml
  id: [kebab-case-id]
  title: [Clear Title]
  sidebar_position: [Sequential Number]
  ```
- **Callouts**: Use Docusaurus admonitions for critical info:
  - `:::note` for extra info.
  - `:::tip` for helpful shortcuts.
  - `:::info` for general background.
  - `:::warning` for dangerous hardware steps.
  - `:::danger` for critical safety warnings.

## 3. RAG Optimization
- **Headings**: Use clear, descriptive H2 and H3 tags to allow the RAG system to chunk content effectively.
- **Context**: Avoid "this" or "that" when referring to subjects; be explicit so chunks are self-contained.
- **Metadata**: Include keywords in frontmatter if the topic is specialized.

## 4. Visuals & Code
- **Mermaid**: Use Mermaid diagrams for flowcharts.
- **Code Blocks**: Always include language tags and comments for complex logic.

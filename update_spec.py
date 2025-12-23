# -*- coding: utf-8 -*-
import re

file_path = "specs/001-intro-physical-ai-humanoid/spec.md"

with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Change 1: Add new clarification bullet
old_text = """- Q: What is the target length/reading time for the introduction to ensure it provides adequate foundation without overwhelming readers? → A: Comprehensive: 10-15 pages (2500-4000 words, 15-25 minute read)

## User Scenarios & Testing"""

new_text = """- Q: What is the target length/reading time for the introduction to ensure it provides adequate foundation without overwhelming readers? → A: Comprehensive: 10-15 pages (2500-4000 words, 15-25 minute read)
- Q: How should the introduction accommodate readers with varying levels of AI and robotics background (from complete beginners to experienced practitioners)? → A: Tiered content approach: core concepts mandatory for all, with "deep dive" callout boxes for advanced readers and "prerequisite refresher" boxes for beginners

## User Scenarios & Testing"""

content = content.replace(old_text, new_text)

# Change 2: Update edge case
old_edge = "- What happens when a reader has no prior AI or robotics background? (Introduction should establish sufficient baseline knowledge or point to prerequisites)"

new_edge = "- What happens when a reader has no prior AI or robotics background? → Addressed via 'Prerequisite Refresher' callout boxes that provide foundational context without disrupting main narrative flow"

content = content.replace(old_edge, new_edge)

# Change 3: Add FR-012 after FR-011
old_fr_011 = "- **FR-011**: Introduction MUST be comprehensive yet focused, targeting 2500-4000 words (approximately 10-15 pages, 15-25 minute read) to provide thorough foundation without overwhelming readers\n\n### Key Entities"

new_fr_012 = "- **FR-011**: Introduction MUST be comprehensive yet focused, targeting 2500-4000 words (approximately 10-15 pages, 15-25 minute read) to provide thorough foundation without overwhelming readers\n- **FR-012**: Introduction MUST use a tiered content approach with core concepts in the main narrative, supplemented by \"Deep Dive\" callout boxes for advanced readers and \"Prerequisite Refresher\" boxes for beginners\n\n### Key Entities"

content = content.replace(old_fr_011, new_fr_012)

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)

print("File updated successfully")

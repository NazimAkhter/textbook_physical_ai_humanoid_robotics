# Authoritative Sources and APA Citations

**Purpose**: Research and prepare 2-4 APA citations for Physical AI definitions and VLA systems
**Date**: 2025-12-23
**Feature**: 001-intro-physical-ai-humanoid

## Primary Sources for Physical AI

### 1. ROS 2 (Robot Operating System 2)

**Citation**:
Open Robotics. (2024). *ROS 2 Documentation: Jazzy Jalisco*. Open Robotics Foundation. https://docs.ros.org/en/jazzy/

**Relevance**: Authoritative source for ROS 2 architecture, middleware concepts, and robotics software standards. Essential for explaining the "Robotic Nervous System" technology stack component.

**Key Concepts to Reference**:
- Middleware layer and DDS protocol
- Nodes, topics, services, actions architecture
- Real-time capabilities and QoS policies
- Industry adoption as standard

---

### 2. NVIDIA Isaac Platform

**Citation**:
NVIDIA Corporation. (2024). *NVIDIA Isaac Platform for Robotics*. NVIDIA Developer Documentation. https://developer.nvidia.com/isaac

**Relevance**: Official documentation for Isaac Sim, Isaac Gym, and Isaac ROS. Authoritative source for simulation, synthetic data generation, and GPU-accelerated robotics development.

**Key Concepts to Reference**:
- High-fidelity physics simulation
- Photorealistic rendering for computer vision
- Reinforcement learning in simulation (Isaac Gym)
- Sim-to-real transfer capabilities

---

### 3. Vision-Language-Action (VLA) Systems

**Citation**:
Brohan, A., Brown, N., Carbajal, J., Chebotar, Y., Chen, X., Choromanski, K., Ding, T., Driess, D., Dubey, A., Finn, C., Florence, P., Fu, C., Arenas, M. G., Gopalakrishnan, K., Han, K., Hausman, K., Herzog, A., Hsu, J., Ichter, B., ... & Zeng, A. (2023). RT-2: Vision-Language-Action models transfer web knowledge to robotic control. *arXiv preprint arXiv:2307.15818*. https://arxiv.org/abs/2307.15818

**Relevance**: Seminal paper on Vision-Language-Action models demonstrating how large language models can be adapted for robotic control. Introduces the concept of mapping visual observations and language instructions to robot actions.

**Key Concepts to Reference**:
- Multimodal learning (vision + language → actions)
- Transfer learning from web-scale data to robotics
- Generalization to novel objects and instructions
- Zero-shot task execution

**Alternative/Supplementary**:
Driess, D., Xia, F., Sajjadi, M. S., Lynch, C., Chowdhery, A., Ichter, B., Wahid, A., Tompson, J., Vuong, Q., Yu, T., Huang, W., Chebotar, Y., Sermanet, P., Duckworth, D., Levine, S., Vanhoucke, V., Hausman, K., Toussaint, M., Greff, K., ... & Florence, P. (2023). PaLM-E: An embodied multimodal language model. *arXiv preprint arXiv:2303.03378*. https://arxiv.org/abs/2303.03378

---

### 4. Physical AI and Embodied Intelligence

**Citation**:
Pfeifer, R., & Bongard, J. (2006). *How the body shapes the way we think: A new view of intelligence*. MIT Press.

**Relevance**: Foundational text on embodied cognition theory and how physical embodiment is essential for intelligence. Provides philosophical and theoretical grounding for Physical AI concepts.

**Key Concepts to Reference**:
- Embodied cognition vs. traditional symbolic AI
- Morphological computation
- Sensorimotor coordination
- Intelligence arising from body-environment interaction

**Alternative/Supplementary (More Recent)**:
Brooks, R. A. (1991). Intelligence without representation. *Artificial Intelligence, 47*(1-3), 139-159. https://doi.org/10.1016/0004-3702(91)90053-M

---

## Additional Supporting Sources

### Digital Twin Technology

Grieves, M., & Vickers, J. (2017). Digital twin: Mitigating unpredictable, undesirable emergent behavior in complex systems. In *Transdisciplinary perspectives on complex systems* (pp. 85-113). Springer, Cham.

### Robotics and AI Integration

Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic robotics*. MIT Press. (Classic reference for robot perception and navigation)

### Human-Robot Interaction

Dautenhahn, K. (2007). Socially intelligent robots: dimensions of human–robot interaction. *Philosophical Transactions of the Royal Society B: Biological Sciences, 362*(1480), 679-704.

---

## Citation Usage Strategy

### For Introduction Chapter:

**Minimal Citation Approach** (2-4 citations inline):
1. **ROS 2 (Open Robotics, 2024)**: When introducing ROS 2 as middleware standard
2. **NVIDIA Isaac (NVIDIA, 2024)**: When explaining simulation platform
3. **RT-2 or PaLM-E (Brohan et al., 2023 or Driess et al., 2023)**: When introducing VLA systems
4. **Embodied Cognition (Pfeifer & Bongard, 2006)**: Optional - in Deep Dive callout for theoretical grounding

### Inline Citation Examples:

```markdown
ROS 2 has emerged as the de facto standard middleware for robotics development,
providing a modular architecture for building distributed robot systems (Open Robotics, 2024).

Recent advances in Vision-Language-Action (VLA) models demonstrate how large language
models can be adapted for robotic control, enabling robots to understand natural
language instructions and execute corresponding physical actions (Brohan et al., 2023).
```

---

## References Section (APA 7th Edition)

**To be added at end of introduction.md**:

```markdown
## References

Brohan, A., Brown, N., Carbajal, J., Chebotar, Y., Chen, X., Choromanski, K., Ding, T., Driess, D., Dubey, A., Finn, C., Florence, P., Fu, C., Arenas, M. G., Gopalakrishnan, K., Han, K., Hausman, K., Herzog, A., Hsu, J., Ichter, B., ... & Zeng, A. (2023). RT-2: Vision-Language-Action models transfer web knowledge to robotic control. *arXiv preprint arXiv:2307.15818*. https://arxiv.org/abs/2307.15818

NVIDIA Corporation. (2024). *NVIDIA Isaac Platform for Robotics*. NVIDIA Developer Documentation. https://developer.nvidia.com/isaac

Open Robotics. (2024). *ROS 2 Documentation: Jazzy Jalisco*. Open Robotics Foundation. https://docs.ros.org/en/jazzy/

Pfeifer, R., & Bongard, J. (2006). *How the body shapes the way we think: A new view of intelligence*. MIT Press.
```

---

## Notes

- All URLs verified as of 2025-12-23
- ROS 2 documentation uses latest LTS version (Jazzy Jalisco)
- NVIDIA Isaac documentation is current platform version
- RT-2 paper is most recent major VLA publication (July 2023)
- Pfeifer & Bongard provides classic theoretical foundation

**Action Items**:
- Verify all URLs are accessible before final publication
- Update version numbers if newer releases published
- Consider adding DOI links where available for academic papers

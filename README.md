# dtsim-lorawan
A simple discrete-time simulator for cyber-physical systems research in LoRaWAN, originally written for the validation of the idea proposed in the following research paper:

Chao Wang, Cheng-Hsun Chuang, Yu-Wei Chen, and Yun-Fan Chen. 2024. On Cyber-Physical Fault Resilience in Data Communication: A Case From A LoRaWAN Network Systems Design. ACM Trans. Cyber-Phys. Syst. 8, 3, Article 36 (July 2024), 25 pages. https://doi.org/10.1145/3639571

Now, it is undergoing some updates for a new, related research project.

Naming convention of the dataset file for dtsim.cpp: s*d*f*j*. Each star symbol is an integer number.
- s: SETTING_TYPE;
- d: EVALUATION_TYPE (for mobile gateway prediction);
- f: number of faulty areas at any moment;
- j: whether there is an attacker (jammer); either 0 or 1.



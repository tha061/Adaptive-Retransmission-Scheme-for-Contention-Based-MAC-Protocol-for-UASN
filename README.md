# Guidelines for the implementation of adaptive retransmission scheme for contention-based protocols including Aloha and CSMA.

### Brief description: 
Due to the limited capacity and high propagation delay of underwater communication channels, contention-based media access control
(MAC) protocols suffer from a low packet delivery ratio (PDR) and a high end-to-end (E2E) delay in underwater acoustic sensor networks due to
the reliance on packet retransmission for reliable data delivery. In order to address the problem of low performance, we propose a novel adaptive
retransmission scheme, named ARS, which dynamically selects an optimal value of the maximum number of retransmissions, such that the successful
delivery probability of a packet is maximized for a given network load. ARS can be used for various contention-based protocols and hybrid MAC
protocols that have contention periods. ARS is applied to well-known contention-based protocols, Aloha and CSMA. Simulation results
show that ARS can achieve signiﬁcant performance improvement in terms of PDR and E2E delay over original MAC protocols.


### DATE OF CURRENT VERSION (V1.0.0): Jan. 2014

### CONTENTS: 
C++ implementation of adaptive retransmission scheme for Aloha protocol

C++ implementation of adaptive retransmission scheme for CSMA protocol

### RELATED PUBLICATION: 
Thi-Tham Nguyen and Seokhoon Yoon. 2015. ARS: an adaptive retransmission scheme for contention-based MAC protocols in underwater acoustic sensor networks. Int. J. Distrib. Sen. Netw. 2015, Article 16. DOI:https://doi.org/10.1155/2015/826263

### NOTICE:
The code was built and tested on DESERT (Design, Simulate, Emulate and Realize Test-beds) underwater simulation framework based on NS2 Miracle is used to simulate
the protocols in a realistic underwater communication environment. 

### BUG REPORTS: 
If you find any bug, please send your feedback to nttham0611@gmail.com 

### LICENSE: 
The code is licensed under Copyright (c) 2012 Regents of the SIGNET lab, University of Padova. All rights reserved.


### If you use our code in your research and/or software, we would appreciate citations to the following paper:

Thi-Tham Nguyen and Seokhoon Yoon. 2015. ARS: an adaptive retransmission scheme for contention-based MAC protocols in underwater acoustic sensor networks. Int. J. Distrib. Sen. Netw. 2015, Article 16. DOI:https://doi.org/10.1155/2015/826263

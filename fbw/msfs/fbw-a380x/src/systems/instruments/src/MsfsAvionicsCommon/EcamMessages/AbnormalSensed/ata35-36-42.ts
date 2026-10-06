// Copyright (c) 2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { AbnormalProcedure } from '..';

// Convention for IDs:
// First two digits: ATA chapter
// Third digit: Sub chapter, if needed
// Fourth digit:
//    0 for MEMOs,
//    1 for normal checklists,
//    2 for infos,
//    3 for INOP SYS,
//    4 for limitations,
//    7 for deferred procedures,
//    8 for ABN sensed procedures,
//    9 for ABN non-sensed procedures

/** All abnormal sensed procedures (alerts, via ECL) should be here. */
export const EcamAbnormalSensedAta353642: { [n: number]: AbnormalProcedure } = {
  361800001: {
    title: '\x1b<4m\x1b4mAIR\x1bm XBLEED FAULT',
    sensed: true,
    items: [],
  },
  361800002: {
    title: '\x1b<4m\x1b4mAIR\x1bm ENG 1(4) BLEED LEAK',
    sensed: true,
    items: [],
  },
  361800003: {
    title: '\x1b<4m\x1b4mAIR\x1bm ENG 2(3) BLEED LEAK',
    sensed: true,
    items: [],
  },
  361800004: {
    title: '\x1b<4m\x1b4mAIR\x1bm ENG HP VLV NOT OPEN',
    sensed: true,
    items: [],
  },
  361800005: {
    title: '\x1b<4m\x1b4mAIR\x1bm ENG BLEED FAULT',
    sensed: true,
    items: [],
  },
};

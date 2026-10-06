// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import type { ChecklistSpecialItem } from './index';

export enum ChecklistLineStyle {
  Standard = 'Standard',
  Cyan = 'Cyan',
  Green = 'Green',
  Amber = 'Amber',
  White = 'White',
  Red = 'Red',
  Headline = 'Headline',
  SubHeadline = 'SubHeadline',
  CenteredSubHeadline = 'CenteredSubHeadline',
  SeparationLine = 'SeparationLine',
  Empty = 'Empty',
  ChecklistItem = 'ChecklistItem',
  CompletedChecklist = 'CompletedChecklist',
  CompletedDeferredProcedure = 'CompletedDeferredProcedure',
  DeferredProcedure = 'DeferredProcedure',
  OmissionDots = 'OmissionDots',
  LandAsap = 'LandAsap',
  LandAnsa = 'LandAnsa',
  ChecklistCondition = 'ChecklistCondition',
  ChecklistItemInactive = 'ChecklistItemInactive',
}

export enum DeferredProcedureType {
  ALL_PHASES,
  AT_TOP_OF_DESCENT,
  FOR_APPROACH,
  FOR_LANDING,
}
export const DEFERRED_PROCEDURE_TYPE_TO_STRING = ['ALL PHASES', 'AT TOP OF DESCENT', 'FOR APPROACH', 'FOR LANDING'];

export enum WdSpecialLine {
  ClComplete,
  Reset,
  Clear,
  Empty,
  SeparationLine,
}

export const FMS_PRED_UNRELIABLE_CHECKLIST_ITEM: ChecklistSpecialItem = {
  name: 'FMS PRED UNRELIABLE', // TODO Replace with FMS PRED UNRELIABLE WITHOUT ACCURATE FMS FUEL PENALTY INSERTION once multiple lines supported
  sensed: false,
  style: ChecklistLineStyle.ChecklistCondition,
};

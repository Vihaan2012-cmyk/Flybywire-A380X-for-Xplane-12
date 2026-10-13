//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

export class ObjectUtils {
  public static fromEntries<T = any>(entries: Iterable<readonly [PropertyKey, T]>): Record<PropertyKey, T> {
    const result: Record<PropertyKey, T> = {};
    for (const [key, value] of entries) {
      result[key] = value;
    }
    return result;
  }
}

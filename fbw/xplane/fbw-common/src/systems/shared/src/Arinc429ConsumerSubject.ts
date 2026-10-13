//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { AbstractSubscribable, Consumer, ReadonlyLifecycle, Subscription } from '@microsoft/msfs-sdk';
import { Arinc429Register, Arinc429WordData } from './arinc429';

type Arinc429Input = Arinc429WordData;

export class Arinc429ConsumerSubject extends AbstractSubscribable<Arinc429WordData> {
  public readonly canInitialNotify = true;

  private readonly word = Arinc429Register.empty();

  private consumerSub: Subscription | undefined;

  private paused = false;

  private alive = true;

  private constructor(consumer: Consumer<Arinc429Input> | null | undefined) {
    super();
    this.setConsumer(consumer ?? null);
  }

  public static create(consumer: Consumer<Arinc429Input> | null | undefined): Arinc429ConsumerSubject {
    return new Arinc429ConsumerSubject(consumer);
  }

  public get isAlive(): boolean {
    return this.alive;
  }

  public get isPaused(): boolean {
    return this.paused;
  }

  public get(): Arinc429WordData {
    return this.word;
  }

  public setConsumer(consumer: Consumer<Arinc429Input> | null): this {
    if (!this.alive) {
      return this;
    }
    this.consumerSub?.destroy();
    this.consumerSub = consumer ? consumer.handle((value) => this.onValue(value), this.paused) : undefined;
    return this;
  }

  public reset(initialVal: Arinc429Input, consumer: Consumer<Arinc429Input> | null = null): this {
    this.consumerSub?.destroy();
    this.consumerSub = undefined;
    this.onValue(initialVal);
    return this.setConsumer(consumer);
  }

  public pause(): this {
    if (!this.paused) {
      this.paused = true;
      this.consumerSub?.pause();
    }
    return this;
  }

  public resume(): this {
    if (this.paused) {
      this.paused = false;
      this.consumerSub?.resume(true);
    }
    return this;
  }

  public withLifecycle(lifecycle: ReadonlyLifecycle): this {
    lifecycle.register(this);
    return this;
  }

  public destroy(): void {
    this.alive = false;
    this.consumerSub?.destroy();
    this.consumerSub = undefined;
  }

  private onValue(input: Arinc429Input): void {
    if (input.value !== this.word.value || input.ssm !== this.word.ssm) {
      this.word.set(input.rawWord);
      this.notify();
    }
  }
}

export class Arinc429LocalVarConsumerSubject extends AbstractSubscribable<Arinc429WordData> {
  public readonly canInitialNotify = true;

  private readonly word = Arinc429Register.empty();

  private consumerSub: Subscription | undefined;

  private paused = false;

  private alive = true;

  private constructor(consumer: Consumer<number> | null | undefined, initialVal: number | null) {
    super();
    if (initialVal !== null) {
      this.word.set(initialVal);
    }
    this.setConsumer(consumer ?? null);
  }

  public static create(
    consumer: Consumer<number> | null | undefined,
    initialVal: number | null = 0,
  ): Arinc429LocalVarConsumerSubject {
    return new Arinc429LocalVarConsumerSubject(consumer, initialVal);
  }

  public get isAlive(): boolean {
    return this.alive;
  }

  public get isPaused(): boolean {
    return this.paused;
  }

  public get(): Arinc429WordData {
    return this.word;
  }

  public setConsumer(consumer: Consumer<number> | null): this {
    if (!this.alive) {
      return this;
    }
    this.consumerSub?.destroy();
    this.consumerSub = consumer ? consumer.handle((value) => this.onValue(value), this.paused) : undefined;
    return this;
  }

  public pause(): this {
    if (!this.paused) {
      this.paused = true;
      this.consumerSub?.pause();
    }
    return this;
  }

  public resume(): this {
    if (this.paused) {
      this.paused = false;
      this.consumerSub?.resume(true);
    }
    return this;
  }

  public withLifecycle(lifecycle: ReadonlyLifecycle): this {
    lifecycle.register(this);
    return this;
  }

  public destroy(): void {
    this.alive = false;
    this.consumerSub?.destroy();
    this.consumerSub = undefined;
  }

  private onValue(raw: number): void {
    if (raw !== this.word.rawWord) {
      this.word.set(raw);
      this.notify();
    }
  }
}

import { strict as assert } from 'node:assert';
import {
  ArraySubject, ConsumerSubject, MapSubject, MappedSubject, SetSubject, Subject, SubEvent,
  SubscribableArrayEventType, SubscribableMapEventType, SubscribableUtils,
} from '../index';

const log: any[] = [];
const s = Subject.create(1);
const sub = s.sub((v) => log.push(v), true);
s.set(2); s.set(2); s.set(3);
assert.deepEqual(log, [1, 2, 3]);
sub.pause(); s.set(4); sub.resume(true);
assert.deepEqual(log, [1, 2, 3, 4]);
sub.destroy(); s.set(5);
assert.equal(log.length, 4);
assert.throws(() => sub.pause());

const a = Subject.create(1), b = Subject.create(2);
const sum = MappedSubject.create(([x, y]) => x + y, a, b);
const seen: number[] = [];
sum.sub((v) => seen.push(v), true);
a.set(10);
assert.deepEqual(seen, [3, 12]);
sum.pause(); a.set(20); b.set(30); assert.equal(sum.get(), 12); sum.resume();
assert.equal(sum.get(), 50);
sum.destroy(); a.set(0); assert.equal(sum.get(), 50);
const tuple = MappedSubject.create(a, b);
assert.deepEqual([...tuple.get()], [0, 30]);

const m = a.map((x) => x * 2);
assert.equal(m.get(), 0);
a.set(4); assert.equal(m.get(), 8);

const t = Subject.create(0);
a.pipe(t, (v) => v + 1);
assert.equal(t.get(), 5); a.set(6); assert.equal(t.get(), 7);

const arr = ArraySubject.create<number>([1]);
const ev: any[] = [];
arr.sub((i, type, item) => ev.push([i, type, item]), true);
arr.insert(2); arr.removeAt(0); arr.set([7, 8]);
assert.deepEqual(ev.map((e) => e[1]), ['Added', 'Added', 'Removed', 'Cleared', 'Added']);
assert.deepEqual(arr.getArray(), [7, 8]);
assert.throws(() => arr.get(9));

const mp = MapSubject.create<string, number>();
const me: string[] = [];
mp.sub((_m, type, k) => me.push(type + k));
mp.setValue('a', 1); mp.setValue('a', 2); mp.setValue('b', 1); mp.delete('a');
assert.deepEqual(me, ['Addeda', 'Changeda', 'Addedb', 'Deleteda']);
const mp2 = MapSubject.create<string, number>();
mp.pipe(mp2);
assert.equal(mp2.getValue('b'), 1); mp.setValue('c', 3); assert.equal(mp2.size, 2);

const st = SetSubject.create([1, 2]);
const st2 = SetSubject.create<number>();
st.pipe(st2, (k) => k % 2);
st.add(3); st.delete(1);
assert.deepEqual([...st2.get()].sort(), [0, 1]);
st.delete(3);
assert.deepEqual([...st2.get()], [0]);

const evn = new SubEvent<number, string>();
const got: string[] = [];
const es = evn.on((snd, d) => got.push(snd + d));
evn.notify(1, 'x'); es.destroy(); evn.notify(2, 'y');
assert.deepEqual(got, ['1x']);

const cb: any = { handle: (h: any) => { h(42); return { pause() {return this;}, resume() {return this;}, destroy() {}, isAlive: true, isPaused: false } } };
assert.equal(ConsumerSubject.create(cb, 0).get(), 42);
assert.equal(SubscribableUtils.toSubscribable(5, true).get(), 5);
assert.ok(SubscribableUtils.isMutableSubscribable(s));

const src = Subject.create(1);
const tgt = Subject.create(0);
const pm = SubscribableUtils.pipeMappedSource(src, tgt, (v) => Subject.create(v * 10));
assert.equal(tgt.get(), 10); src.set(2); assert.equal(tgt.get(), 20);
pm.destroy(); src.set(3); assert.equal(tgt.get(), 20);
console.log('sub tests ok');

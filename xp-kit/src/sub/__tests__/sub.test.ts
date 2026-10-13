import { strict as assert } from 'node:assert';
import {
  ListCell, FeedCell, DictCell, DerivedCell, BagCell, Cell, Notice,
  WatchableArraySignalType, WatchableMapSignalType, Watchables,
} from '../index';

const log: any[] = [];
const s = Cell.create(1);
const sub = s.sub((v) => log.push(v), true);
s.set(2); s.set(2); s.set(3);
assert.deepEqual(log, [1, 2, 3]);
sub.pause(); s.set(4); sub.resume(true);
assert.deepEqual(log, [1, 2, 3, 4]);
sub.destroy(); s.set(5);
assert.equal(log.length, 4);
assert.throws(() => sub.pause());

const a = Cell.create(1), b = Cell.create(2);
const sum = DerivedCell.create(([x, y]) => x + y, a, b);
const seen: number[] = [];
sum.sub((v) => seen.push(v), true);
a.set(10);
assert.deepEqual(seen, [3, 12]);
sum.pause(); a.set(20); b.set(30); assert.equal(sum.get(), 12); sum.resume();
assert.equal(sum.get(), 50);
sum.destroy(); a.set(0); assert.equal(sum.get(), 50);
const tuple = DerivedCell.create(a, b);
assert.deepEqual([...tuple.get()], [0, 30]);

const m = a.map((x) => x * 2);
assert.equal(m.get(), 0);
a.set(4); assert.equal(m.get(), 8);

const t = Cell.create(0);
a.pipe(t, (v) => v + 1);
assert.equal(t.get(), 5); a.set(6); assert.equal(t.get(), 7);

const arr = ListCell.create<number>([1]);
const ev: any[] = [];
arr.sub((i, type, item) => ev.push([i, type, item]), true);
arr.insert(2); arr.removeAt(0); arr.set([7, 8]);
assert.deepEqual(ev.map((e) => e[1]), ['Added', 'Added', 'Removed', 'Cleared', 'Added']);
assert.deepEqual(arr.items(), [7, 8]);
assert.throws(() => arr.get(9));

const mp = DictCell.create<string, number>();
const me: string[] = [];
mp.sub((_m, type, k) => me.push(type + k));
mp.setValue('a', 1); mp.setValue('a', 2); mp.setValue('b', 1); mp.delete('a');
assert.deepEqual(me, ['Addeda', 'Changeda', 'Addedb', 'Deleteda']);
const mp2 = DictCell.create<string, number>();
mp.pipe(mp2);
assert.equal(mp2.getValue('b'), 1); mp.setValue('c', 3); assert.equal(mp2.size, 2);

const st = BagCell.create([1, 2]);
const st2 = BagCell.create<number>();
st.pipe(st2, (k) => k % 2);
st.add(3); st.delete(1);
assert.deepEqual([...st2.get()].sort(), [0, 1]);
st.delete(3);
assert.deepEqual([...st2.get()], [0]);

const evn = new Notice<number, string>();
const got: string[] = [];
const es = evn.on((snd, d) => got.push(snd + d));
evn.notify(1, 'x'); es.destroy(); evn.notify(2, 'y');
assert.deepEqual(got, ['1x']);

const cb: any = { handle: (h: any) => { h(42); return { pause() {return this;}, resume() {return this;}, destroy() {}, isAlive: true, isPaused: false } } };
assert.equal(FeedCell.create(cb, 0).get(), 42);
assert.equal(Watchables.asWatchable(5, true).get(), 5);
assert.ok(Watchables.isWritable(s));

const src = Cell.create(1);
const tgt = Cell.create(0);
const pm = Watchables.pipeMappedSource(src, tgt, (v) => Cell.create(v * 10));
assert.equal(tgt.get(), 10); src.set(2); assert.equal(tgt.get(), 20);
pm.destroy(); src.set(3); assert.equal(tgt.get(), 20);
console.log('sub tests ok');

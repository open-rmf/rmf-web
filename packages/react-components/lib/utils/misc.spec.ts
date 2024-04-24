import { almostShallowEqual } from '.';
import { defaultDict } from './misc';

it('sets correct default value', () => {
  const dict = defaultDict(() => 'hello');
  expect('something' in dict).toBeFalsy();
  expect(dict['something']).toBe('hello');
});

it('assigning before any lookup assigns the given value', () => {
  const dict = defaultDict(() => 'hello');
  dict['world'] = 'world';
  expect(dict['world']).toBe('world');
});

it('assigning overwrites previous value', () => {
  const dict = defaultDict(() => 'hello');
  dict['world'] = 'world';
  expect(dict['world']).toBe('world');
  dict['world'] = '!';
  expect(dict['world']).toBe('!');
});

describe('almostShallowEqual', () => {
  it('smoke test', () => {
    // `foo` is not strict equal even though they contain objects with the same content.
    const a = { foo: { bar: 'baz' } };
    const b = { foo: { bar: 'baz' } };
    expect(almostShallowEqual(a, b)).toBe(false);
    // this should traverse into `foo` and check the contents shallowing, rather than checking
    // `foo` itself.
    expect(almostShallowEqual(a, b, ['foo'])).toBe(true);
  });
});

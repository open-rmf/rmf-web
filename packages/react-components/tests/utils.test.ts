import { defaultDict } from '../lib';

test('sets correct default value', () => {
  const dict = defaultDict(() => 'hello');
  expect('something' in dict).toBeFalsy();
  expect(dict['something']).toBe('hello');
});

test('assigning before any lookup assigns the given value', () => {
  const dict = defaultDict(() => 'hello');
  dict['world'] = 'world';
  expect(dict['world']).toBe('world');
});

test('assigning overwrites previous value', () => {
  const dict = defaultDict(() => 'hello');
  dict['world'] = 'world';
  expect(dict['world']).toBe('world');
  dict['world'] = '!';
  expect(dict['world']).toBe('!');
});

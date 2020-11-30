import { StackNavigator } from '../lib';

let stack: StackNavigator<number>;

beforeEach(() => {
  stack = new StackNavigator(0);
});

test('push', () => {
  stack.push(10);
  expect(stack.stack).toHaveLength(2);
  expect(stack.stack[0]).toBe(0);
  expect(stack.stack[1]).toBe(10);
});

test('pop does not remove last item', () => {
  stack.push(2);
  stack.pop();
  stack.pop();
  expect(stack.stack).toHaveLength(1);
  expect(stack.stack[0]).toBe(0);
});

test('reset leaves only the home item in the stack', () => {
  stack.push(2);
  stack.push(3);
  expect(stack.stack).toHaveLength(3);
  stack.reset();
  expect(stack.stack).toHaveLength(1);
  expect(stack.stack[0]).toBe(0);
});

import { StackNavigator } from '../lib';

describe('Stack navigator', () => {
  let stack: StackNavigator<number>;

  beforeEach(() => {
    stack = new StackNavigator(0);
  });

  it('push', () => {
    stack.push(10);
    expect(stack.stack).toHaveSize(2);
    expect(stack.stack[0]).toBe(0);
    expect(stack.stack[1]).toBe(10);
  });

  it('pop does not remove last item', () => {
    stack.push(2);
    stack.pop();
    stack.pop();
    expect(stack.stack).toHaveSize(1);
    expect(stack.stack[0]).toBe(0);
  });

  it('reset leaves only the home item in the stack', () => {
    stack.push(2);
    stack.push(3);
    expect(stack.stack).toHaveSize(3);
    stack.reset();
    expect(stack.stack).toHaveSize(1);
    expect(stack.stack[0]).toBe(0);
  });
});

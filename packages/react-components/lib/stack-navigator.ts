export class StackNavigator<KeyType extends string | number> {
  stack: KeyType[];

  constructor(home: KeyType) {
    this.stack = [home];
  }

  push(viewId: KeyType): void {
    this.stack.push(viewId);
  }

  pop(): KeyType {
    this.stack.length > 1 && this.stack.pop();
    return this.stack[this.stack.length - 1];
  }

  reset(): KeyType {
    this.stack = [this.stack[0]];
    return this.stack[0];
  }

  top(): KeyType {
    return this.stack[this.stack.length - 1];
  }
}

export default StackNavigator;

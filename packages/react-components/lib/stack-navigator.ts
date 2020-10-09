export interface StackNavigatorProps {
  variant?: 'backHome' | 'backHomeClose';
  onBack(): void;
  onHome(): void;
  onClose(): void;
}

export default class StackNavigator {
  stack: (number | string)[];

  constructor(home: number | string) {
    this.stack = [home];
  }

  push(viewId: number): void {
    this.stack.push(viewId);
  }

  pop(): number | string {
    this.stack.length > 1 && this.stack.pop();
    return this.stack[this.stack.length - 1];
  }

  reset(): number | string {
    this.stack = [this.stack[0]];
    return this.stack[0];
  }
}

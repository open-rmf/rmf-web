export interface StackNavigatorProps {
  variant?: 'backHome' | 'backHomeClose';
  onBack(): void;
  onHome(): void;
  onClose(): void;
}

export default class StackNavigator {
  constructor(home: number | string) {
    this._stack = [home];
  }

  push(viewId: number): void {
    this._stack.push(viewId);
  }

  pop(): number | string {
    this._stack.length > 1 && this._stack.pop();
    return this._stack[this._stack.length - 1];
  }

  reset(): number | string {
    this._stack = [this._stack[0]];
    return this._stack[0];
  }

  private _stack: (number | string)[];
}

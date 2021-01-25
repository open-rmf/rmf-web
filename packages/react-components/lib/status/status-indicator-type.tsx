// TODO - change typing if backend data is different
export interface ItemIndicator {
  state: boolean;
}

export interface StatusIndicator {
  [key: string]: { [key: string]: ItemIndicator };
}

export interface ItemIndicator {
  state: boolean;
}

export interface StatusIndicator {
  [key: string]: { [key: string]: ItemIndicator };
}

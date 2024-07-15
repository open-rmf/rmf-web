let id = 0;
export function uniqueId(): string {
  return (id++).toString();
}

// colors used outside material ui
export const colorPalette: { [key: string]: string } = {
  unknown: '#cccccc',
};

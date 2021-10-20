import { WindowLayout } from './window-manager';

export const makeLayouts: () => WindowLayout[] = () => [
  { i: '0', x: 0, y: 0, w: 4, h: 4, minW: 4, minH: 4 },
  { i: '1', x: 4, y: 0, w: 4, h: 4, minW: 4, minH: 4 },
  { i: '2', x: 0, y: 4, w: 4, h: 4, minW: 4, minH: 4 },
  { i: '3', x: 4, y: 4, w: 4, h: 4, minW: 4, minH: 4 },
];

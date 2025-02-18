import { vi } from 'vitest';

/* @ts-ignore */
HTMLCanvasElement.prototype.getContext = () => {
  return {
    fillStyle: '',
    fillRect: vi.fn(),
  };
};

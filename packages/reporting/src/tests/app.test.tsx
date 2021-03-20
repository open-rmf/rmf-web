import React from 'react';
import { render } from '@testing-library/react';
import App from '../components/app';

test('smoke test', () => {
  render(<App />);
});

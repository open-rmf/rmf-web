import { render } from '@testing-library/react';
import React from 'react';
import { ConfirmationAlertDialog } from '../lib';

test('smoke test', () => {
  render(<ConfirmationAlertDialog open={true} />);
});

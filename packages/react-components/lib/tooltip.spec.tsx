import { Typography } from '@mui/material';
import { fireEvent, render, screen } from '@testing-library/react';

import Tooltip from './tooltip';

describe('Tooltip', () => {
  it('renders the tooltip', () => {
    render(
      <Tooltip title="test" id="test" enabled={true}>
        <Typography variant="h5">Tooltip is enabled</Typography>
      </Tooltip>,
    );

    fireEvent.mouseEnter(screen.getByRole('heading'));

    expect(screen.getByTestId('test-tooltip')).toBeTruthy();
  });

  it('does not render the tooltip when disabled', () => {
    render(
      <Tooltip title="test" id="test" enabled={false}>
        <Typography variant="h5">Tooltip is disabled</Typography>
      </Tooltip>,
    );

    fireEvent.mouseEnter(screen.getByRole('heading'));

    expect(screen.queryByTestId('test-tooltip')).toBeNull();
  });
});

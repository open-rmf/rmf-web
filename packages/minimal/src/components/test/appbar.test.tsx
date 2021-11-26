import { render, screen, waitFor } from '@testing-library/react';
import { AppBar } from '../appbar';

describe('AppBar', () => {
  it('should render app bar properly', async () => {
    render(<AppBar />);
    expect(await waitFor(() => screen.getByText('Minimal App'))).toBeTruthy();
  });
});

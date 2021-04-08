import React from 'react';
import NavigationBar from '../lib/navigation-bar';
import { fireEvent, render, screen } from '@testing-library/react';

describe('Navigation Bar', () => {
  const tabNames = ['Robots', 'Tasks', 'History'];

  it('renders the title bar', () => {
    const root = render(<NavigationBar tabNames={tabNames} />);
    expect(screen.getByLabelText('navigation-tabs')).toBeTruthy();
    root.unmount();
  });

  it('clicking on another tab renders the corresponding tabs panel', () => {
    const root = render(<NavigationBar tabNames={tabNames} />);
    const tab = root.getByText('Tasks');
    fireEvent.click(tab);
    expect(root.getByRole('tabpanel')).toBeTruthy();
    root.unmount();
  });
});

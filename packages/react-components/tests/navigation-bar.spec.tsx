import React from 'react';
import NavigationBar from '../lib/navigation-bar';
import { render, screen } from '@testing-library/react';

describe('Navigation Bar', () => {
  it('renders correctly', () => {
    const tabNames = ['Robots', 'Tasks'];
    const root = render(<NavigationBar tabNames={tabNames} />);
    expect(screen.getByLabelText('navigation-tabs')).toBeTruthy();
    root.unmount();
  });
});

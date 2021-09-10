import { cleanup, render, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { LogoButton } from './logo-button';

describe('LogoButton', () => {
  it('renders and is clickable', () => {
    const mockOnClick = jasmine.createSpy();
    render(
      <LogoButton
        src="/base/test-data/assets/roshealth-logo-white.png"
        alt="logo"
        onClick={mockOnClick}
      />,
    );
    expect(screen.getAllByRole('button').length).toBe(1);
    expect(screen.getByAltText('logo')).toBeTruthy();
    userEvent.click(screen.getByRole('button'));
    expect(mockOnClick).toHaveBeenCalled();
    cleanup();
  });
});

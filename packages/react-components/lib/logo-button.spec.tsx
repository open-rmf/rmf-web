import { cleanup, render, screen, fireEvent } from '@testing-library/react';
import React from 'react';
import { LogoButton } from './logo-button';

describe('LogoButton', () => {
  it('renders and is clickable', () => {
    const mockOnClick = jest.fn();
    render(
      <LogoButton
        src="/base/test-data/assets/roshealth-logo-white.png"
        alt="logo"
        onClick={mockOnClick}
      />,
    );
    expect(screen.getAllByRole('button').length).toBe(1);
    expect(screen.getByAltText('logo')).toBeTruthy();
    fireEvent.click(screen.getByRole('button'));
    expect(mockOnClick).toHaveBeenCalled();
    cleanup();
  });
});

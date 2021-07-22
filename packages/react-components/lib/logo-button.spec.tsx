import { cleanup, render, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { LogoButton } from './logo-button';

describe('LogoImage', () => {
  it('renders and is clickable', () => {
    const mockOnClick = jasmine.createSpy();
    render(
      <LogoButton src="../stories/resources/roshealth-logo-white.png" onClick={mockOnClick} />,
    );
    expect(screen.getAllByRole('img').length).toBe(1);
    expect(screen.getByAltText('logo')).toBeTruthy();
    userEvent.click(screen.getByRole('img'));
    expect(mockOnClick).toHaveBeenCalled();
    cleanup();
  });
});

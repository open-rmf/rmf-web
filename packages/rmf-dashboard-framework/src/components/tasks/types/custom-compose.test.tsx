import { fireEvent, render, screen } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';

import {
  CustomComposeTaskDefinition,
  CustomComposeTaskForm,
  isCustomTaskDescriptionValid,
  makeCustomComposeTaskBookingLabel,
  makeCustomComposeTaskShortDescription,
} from './custom-compose';

describe('Custom compose task form', () => {
  it('Renders custom compose task form', async () => {
    const onChange = vi.fn();
    const onValidate = vi.fn();

    render(<CustomComposeTaskForm taskDesc={''} onChange={onChange} onValidate={onValidate} />);
  });

  it('CustomComposeTaskForm validates input', () => {
    const onChange = vi.fn();
    const onValidate = vi.fn();

    render(<CustomComposeTaskForm taskDesc="" onChange={onChange} onValidate={onValidate} />);

    const textArea = screen.getByRole('textbox', { name: /multiline/i });

    // Invalid input (invalid JSON)
    fireEvent.change(textArea, { target: { value: 'invalid json' } });
    expect(onValidate).toHaveBeenCalledWith(false);

    // Valid input
    const validTaskDesc = '{"valid": "json"}';
    fireEvent.change(textArea, { target: { value: validTaskDesc } });
    expect(onValidate).toHaveBeenCalledWith(true);
  });

  it('Validate description', () => {
    expect(isCustomTaskDescriptionValid('invalid json')).not.toBeTruthy();
    expect(isCustomTaskDescriptionValid('{"valid": "json"}')).toBeTruthy();
  });

  it('Booking label', () => {
    const bookingLabel = makeCustomComposeTaskBookingLabel();
    expect(bookingLabel.task_definition_id).toBe(CustomComposeTaskDefinition.taskDefinitionId);
  });

  it('Short description', () => {
    expect(makeCustomComposeTaskShortDescription('{"valid": "json"}')).toBe('{"valid": "json"}');
  });
});

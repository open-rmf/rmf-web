import { fireEvent, render, screen } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';

import { CustomComposeTaskForm } from './custom-compose';

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
});

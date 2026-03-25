import { render, screen } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';

import { FlexibleTaskForm, mapFormDataToRmfTask } from './flexible-task-form';

describe('FlexibleTaskForm', () => {
  it('renders without crashing initially', () => {
    const mockOnClose = vi.fn();
    const mockOnSuccess = vi.fn();
    render(<FlexibleTaskForm open={true} onClose={mockOnClose} onSuccess={mockOnSuccess} />);
    expect(screen.getByText('Create Flexible Task')).toBeTruthy();
    expect(screen.getByText('Initially blank. Upload schemas to render the form.')).toBeTruthy();
  });

  describe('mapFormDataToRmfTask', () => {
    it('applies basic string interpolation', () => {
      const formData = { taskName: 'patrol', priority: 1 };
      const matchingSchema = { name: '{taskName}', prio: '{priority}' };
      const result = mapFormDataToRmfTask(formData, matchingSchema);
      expect(result).toEqual({ name: 'patrol', prio: 1 });
    });

    it('injects {now} with current timestamp', () => {
      const formData = {};
      const matchingSchema = { startTime: '{now}' };
      const result = mapFormDataToRmfTask(formData, matchingSchema);
      expect(typeof result.startTime).toBe('number');
      // Should be close to Date.now()
      expect(Date.now() - result.startTime).toBeLessThan(100);
    });

    it('handles nested objects', () => {
      const formData = { loc: 'pantry' };
      const matchingSchema = {
        request: {
          category: 'go_to_place',
          description: '{loc}',
        },
      };
      const result = mapFormDataToRmfTask(formData, matchingSchema);
      expect(result).toEqual({
        request: {
          category: 'go_to_place',
          description: 'pantry',
        },
      });
    });

    it('handles ___forEach___ array mapping returning flat items', () => {
      const formData = { locations: ['L1', 'L2'] };
      const matchingSchema = {
        activities: [
          {
            action: '___forEach___',
            source: 'locations',
            template: [{ target: '{item}' }, { target: 'clean_{item}' }],
          },
        ],
      };
      const result = mapFormDataToRmfTask(formData, matchingSchema);
      expect(result).toEqual({
        activities: [
          { target: 'L1' },
          { target: 'clean_L1' },
          { target: 'L2' },
          { target: 'clean_L2' },
        ],
      });
    });

    it('returns unchanged formData if matchingSchema is not provided', () => {
      const formData = { taskName: 'patrol' };
      const result = mapFormDataToRmfTask(formData, null);
      expect(result).toEqual({ taskName: 'patrol' });
    });
  });
});

import { fireEvent, render, screen, within } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';

import {
  addPlaceToPatrolTaskDescription,
  isPatrolTaskDescriptionValid,
  makeDefaultPatrolTaskDescription,
  makePatrolTaskBookingLabel,
  makePatrolTaskShortDescription,
  PatrolTaskDefinition,
  PatrolTaskForm,
} from './patrol';

const mockWaypoints = ['waypoint_1', 'waypoint_2', 'waypoint_3'];

describe('Patrol task form', () => {
  it('PatrolTaskForm renders, changes and validates', async () => {
    const onChange = vi.fn();
    const onValidate = vi.fn();

    render(
      <PatrolTaskForm
        taskDesc={makeDefaultPatrolTaskDescription()}
        patrolWaypoints={mockWaypoints}
        onChange={onChange}
        onValidate={onValidate}
      />,
    );

    const autocomplete = screen.getByTestId('place-name');
    const input = within(autocomplete).getByLabelText(/place name/i);
    autocomplete.focus();
    fireEvent.change(input, { target: { value: 'a' } });
    fireEvent.keyDown(autocomplete, { key: 'ArrowDown' });
    fireEvent.keyDown(autocomplete, { key: 'Enter' });

    expect(onChange).toHaveBeenCalled();
    expect(onValidate).toHaveBeenCalled();
  });

  it('PatrolTaskForm renders and has places', async () => {
    const onChange = vi.fn();
    const onValidate = vi.fn();

    const desc = makeDefaultPatrolTaskDescription();
    const updatedDesc = addPlaceToPatrolTaskDescription(desc, 'waypoint_1');

    const root = render(
      <PatrolTaskForm
        taskDesc={updatedDesc}
        patrolWaypoints={mockWaypoints}
        onChange={onChange}
        onValidate={onValidate}
      />,
    );

    expect(root.getByText(/waypoint_1/i));
  });

  it('booking label', () => {
    let desc = makeDefaultPatrolTaskDescription();
    desc = addPlaceToPatrolTaskDescription(desc, 'waypoint_1');
    let label = makePatrolTaskBookingLabel(desc);
    expect(label.task_definition_id).toBe(PatrolTaskDefinition.taskDefinitionId);
    expect(label.destination).toBe('waypoint_1');

    desc = addPlaceToPatrolTaskDescription(desc, 'waypoint_2');
    label = makePatrolTaskBookingLabel(desc);
    expect(label.task_definition_id).toBe(PatrolTaskDefinition.taskDefinitionId);
    expect(label.destination).toBe('waypoint_2');
  });

  it('validity', () => {
    let desc = makeDefaultPatrolTaskDescription();
    expect(isPatrolTaskDescriptionValid(desc)).not.toBeTruthy();

    desc = addPlaceToPatrolTaskDescription(desc, 'waypoint_1');
    expect(isPatrolTaskDescriptionValid(desc)).toBeTruthy();
  });

  it('short description', () => {
    let desc = makeDefaultPatrolTaskDescription();
    desc = addPlaceToPatrolTaskDescription(desc, 'waypoint_1');
    desc = addPlaceToPatrolTaskDescription(desc, 'waypoint_2');
    expect(makePatrolTaskShortDescription(desc, undefined)).toBe(
      '[Patrol] [1] round/s, along [waypoint_1], [waypoint_2]',
    );
  });
});

import { render, screen } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';

import {
  ComposeCleanTaskDefinition,
  ComposeCleanTaskForm,
  insertCleaningZone,
  isComposeCleanTaskDescriptionValid,
  makeComposeCleanTaskBookingLabel,
  makeComposeCleanTaskShortDescription,
  makeDefaultComposeCleanTaskDescription,
} from './compose-clean';

describe('Compose clean task form', () => {
  it('Renders compose clean task form', () => {
    const onChange = vi.fn();
    const onValidate = vi.fn();

    render(
      <ComposeCleanTaskForm
        taskDesc={makeDefaultComposeCleanTaskDescription()}
        cleaningZones={['test_clean_1', 'test_clean_2']}
        onChange={onChange}
        onValidate={onValidate}
      />,
    );

    expect(screen.getByText('Cleaning Zone')).toBeDefined();
  });

  it('Insert cleaning zone into task description', () => {
    const desc = makeDefaultComposeCleanTaskDescription();
    const insertedDesc = insertCleaningZone(desc, 'clean_zone_3');
    expect(insertedDesc.phases[0].activity.description.activities[0].description).toBe(
      'clean_zone_3',
    );
    expect(
      insertedDesc.phases[0].activity.description.activities[1].description
        .expected_finish_location,
    ).toBe('clean_zone_3');
    expect(
      insertedDesc.phases[0].activity.description.activities[1].description.description.zone,
    ).toBe('clean_zone_3');
  });

  it('Validate task description', () => {
    const desc = makeDefaultComposeCleanTaskDescription();
    const emptyZoneDesc = insertCleaningZone(desc, '');
    expect(isComposeCleanTaskDescriptionValid(emptyZoneDesc)).not.toBeTruthy();
    const insertedDesc = insertCleaningZone(desc, 'clean_zone_3');
    expect(isComposeCleanTaskDescriptionValid(insertedDesc)).toBeTruthy();
  });

  it('Booking label', () => {
    const desc = makeDefaultComposeCleanTaskDescription();
    const insertedDesc = insertCleaningZone(desc, 'clean_zone_3');
    const bookingLabel = makeComposeCleanTaskBookingLabel(insertedDesc);
    expect(bookingLabel.task_definition_id).toBe(ComposeCleanTaskDefinition.taskDefinitionId);
    expect(bookingLabel.destination).toBe('clean_zone_3');
  });

  it('Short description', () => {
    const desc = makeDefaultComposeCleanTaskDescription();
    const insertedDesc = insertCleaningZone(desc, 'clean_zone_3');
    const defaultShortDesc = makeComposeCleanTaskShortDescription(insertedDesc, undefined);
    expect(defaultShortDesc).toBe('[Clean] zone [clean_zone_3]');
  });
});

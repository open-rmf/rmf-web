import { TableContainer, Typography } from '@mui/material';
import React from 'react';
import { ConfirmationDialog, MutexGroupData, MutexGroupGridTable } from 'react-components';
import { AppControllerContext } from './../app-contexts';
import { createMicroApp } from '../micro-app';
import { RmfAppContext } from '../rmf-app';

const RefreshMutexGroupTableInterval = 5000;

export const MutexGroupsApp = createMicroApp('Mutex Groups', () => {
  const rmf = React.useContext(RmfAppContext);
  const appController = React.useContext(AppControllerContext);

  const [mutexGroups, setMutexGroups] = React.useState<Record<string, MutexGroupData>>({});
  const [selectedMutexGroup, setSelectedMutexGroup] = React.useState<MutexGroupData>();
  const [openMutexGroupDialog, setOpenMutexGroupDialog] = React.useState(false);

  const robotIdentifierDelimiter = '/';

  const generateRobotIdentifier = (fleet: string, robot: string) => {
    return `${fleet}${robotIdentifierDelimiter}${robot}`;
  };

  const getFleetFromRobotIdentifier = (robotIdentifier: string) => {
    const split = robotIdentifier.split(robotIdentifierDelimiter);
    if (split.length !== 2) {
      console.error(`Unable to parse fleet from robot identifier: ${robotIdentifier}`);
      return null;
    }
    return split[0];
  };

  const getRobotFromRobotIdentifier = (robotIdentifier: string) => {
    const split = robotIdentifier.split(robotIdentifierDelimiter);
    if (split.length !== 2) {
      console.error(`Unable to parse robot from robot identifier: ${robotIdentifier}`);
      return null;
    }
    return split[1];
  };

  React.useEffect(() => {
    if (!rmf) {
      console.error('Unable to get latest robot information, fleets API unavailable');
      return;
    }

    const refreshMutexGroupTable = async () => {
      const fleets = (await rmf.fleetsApi.getFleetsFleetsGet()).data;
      const updatedMutexGroups: Record<string, MutexGroupData> = {};
      for (const fleet of fleets) {
        if (!fleet.name || !fleet.robots) {
          continue;
        }

        for (const robot of Object.values(fleet.robots)) {
          if (!robot.mutex_groups || !robot.name) {
            continue;
          }
          const robotIdentifier = generateRobotIdentifier(fleet.name, robot.name);

          if (robot.mutex_groups.locked) {
            for (const locked of robot.mutex_groups.locked) {
              if (updatedMutexGroups[locked]) {
                updatedMutexGroups[locked].lockedBy = robotIdentifier;
              } else {
                updatedMutexGroups[locked] = {
                  name: locked,
                  lockedBy: robotIdentifier,
                  requestedBy: [],
                };
              }
            }
          }
          if (robot.mutex_groups.requesting) {
            for (const requesting of robot.mutex_groups.requesting) {
              if (updatedMutexGroups[requesting]) {
                updatedMutexGroups[requesting].requestedBy.push(robotIdentifier);
              } else if (!updatedMutexGroups[requesting].requestedBy) {
                updatedMutexGroups[requesting].requestedBy = [robotIdentifier];
              } else {
                updatedMutexGroups[requesting] = {
                  name: requesting,
                  lockedBy: undefined,
                  requestedBy: [robotIdentifier],
                };
              }
            }
          }
        }
      }
      setMutexGroups(updatedMutexGroups);
    };

    // Initialize table
    (async () => {
      await refreshMutexGroupTable();
    })();

    // Set up regular interval to refresh table
    const refreshInterval = window.setInterval(
      refreshMutexGroupTable,
      RefreshMutexGroupTableInterval,
    );
    return () => {
      clearInterval(refreshInterval);
    };
  }, [rmf]);

  const handleUnlockMutexGroup = React.useCallback<React.MouseEventHandler>(async () => {
    if (!selectedMutexGroup || !selectedMutexGroup.lockedBy) {
      return;
    }
    const fleet = getFleetFromRobotIdentifier(selectedMutexGroup.lockedBy);
    const robot = getRobotFromRobotIdentifier(selectedMutexGroup.lockedBy);
    if (!fleet || !robot) {
      return;
    }

    try {
      if (!rmf) {
        throw new Error('fleets api not available');
      }

      await rmf.fleetsApi?.unlockMutexGroupFleetsNameUnlockMutexGroupPost(
        fleet,
        robot,
        selectedMutexGroup.name,
      );
      appController.showAlert(
        'success',
        `Requested to unlock mutex group ${selectedMutexGroup.name} for ${fleet}:${robot}`,
      );
    } catch (e) {
      appController.showAlert(
        'error',
        `Failed to unlock mutex group ${selectedMutexGroup.name} for ${fleet}:${robot}: ${
          (e as Error).message
        }`,
      );
    }
    setOpenMutexGroupDialog(false);
  }, [selectedMutexGroup, rmf, appController]);

  return (
    <TableContainer>
      <MutexGroupGridTable
        mutexGroups={Object.values(mutexGroups)}
        onMutexGroupClick={(_ev, mutexGroup) => {
          setOpenMutexGroupDialog(true);
          setSelectedMutexGroup(mutexGroup);
        }}
      />
      {openMutexGroupDialog && selectedMutexGroup && (
        <ConfirmationDialog
          confirmText="Confirm unlock"
          cancelText="Cancel"
          open={openMutexGroupDialog}
          title={'Mutex group manual unlock'}
          submitting={undefined}
          onClose={() => {
            setOpenMutexGroupDialog(false);
          }}
          onSubmit={handleUnlockMutexGroup}
        >
          <Typography>
            Confirm unlock mutex group {selectedMutexGroup.name} for{' '}
            {selectedMutexGroup.lockedBy ?? 'n/a'}?
          </Typography>
        </ConfirmationDialog>
      )}
    </TableContainer>
  );
});

import { TableContainer, Typography } from '@mui/material';
import React from 'react';
import { ConfirmationDialog, MutexGroupData, MutexGroupGridTable } from 'react-components';
import { AppEvents } from '../app-events';
import { createMicroApp } from '../micro-app';
import { RmfAppContext } from '../rmf-app';

const RefreshMutexGroupTableInterval = 5000;

export const MutexGroupsApp = createMicroApp('Mutex Groups', () => {
  const rmf = React.useContext(RmfAppContext);

  const [mutexGroups, setMutexGroups] = React.useState<Record<string, MutexGroupData>>({});
  const [selectedMutexGroup, setSelectedMutexGroup] = React.useState<MutexGroupData>();
  const [openMutexGroupDialog, setOpenMutexGroupDialog] = React.useState(false);

  React.useEffect(() => {
    if (!rmf) {
      console.error('Unable to get latest robot information, fleets API unavailable');
      return;
    }

    const refreshMutexGroupTable = async () => {
      const fleets = (await rmf.fleetsApi.getFleetsFleetsGet()).data;
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
    if (!selectedMutexGroup) {
      return;
    }

    // try {
    //   if (!rmf) {
    //     throw new Error('fleets api not available');
    //   }
    //   const resp = await rmf.fleetsApi?.recommissionRobotFleetsNameRecommissionPost(
    //     fleet,
    //     robotState.name,
    //   );
    //   if (!resp.data.commission.success) {
    //     appController.showAlert(
    //       'error',
    //       `Failed to recommission ${fleet}:${robotState.name}: ${
    //         resp.data.commission.errors ?? ''
    //       }`,
    //     );
    //   } else {
    //     appController.showAlert('success', `Recommission of ${fleet}:${robotState.name} requested`);
    //   }
    // } catch (e) {
    //   appController.showAlert(
    //     'error',
    //     `Failed to recommission ${fleet}:${robotState.name}: ${(e as Error).message}`,
    //   );
    // }
    setOpenMutexGroupDialog(false);
  }, [selectedMutexGroup, rmf]);

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
          <Typography>Confirm decommission robot?</Typography>
        </ConfirmationDialog>
      )}
    </TableContainer>
  );
});

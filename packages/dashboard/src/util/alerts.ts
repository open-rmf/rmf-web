import { createMuiTheme } from '@material-ui/core';
import Swal, { SweetAlertIcon } from 'sweetalert2';

export const successMsg = (msg: string) => {
  Swal.fire({
    title: 'Done!',
    text: !!msg ? `${msg}` : 'Successful Operation',
    icon: 'success',
    timer: 2000,
    heightAuto: false,
  });
};

export const errorMsg = (error: string) => {
  Swal.fire({
    title: 'Ups',
    text: !!error ? `${error}` : 'An error has occurred',
    icon: 'error',
    heightAuto: false,
  });
};

const theme = createMuiTheme();

interface VerificationAlertProps {
  title?: string;
  body?: string;
  icon?: SweetAlertIcon;
  confirmButtonText?: string;
  cancelButtonText?: string;
  confirmCallback?: () => void;
  cancelCallback?: () => void;
}

/**
 * Modal to warn a user about something related to an action.
 */
export const verificationAlert = (props: VerificationAlertProps) => {
  const {
    title,
    body,
    icon,
    confirmCallback,
    cancelCallback,
    confirmButtonText,
    cancelButtonText,
  } = props;
  const alertTitle = title || 'Are you sure you want to continue?';
  const alertText = body || 'Once you accept this there is no turning back.';
  const alertIcon = icon || 'warning';
  const alertConfirmButtonText = confirmButtonText || `Yes, I'm sure.`;
  const alertCancelButtonText = cancelButtonText || 'No';

  Swal.fire({
    title: alertTitle,
    text: alertText,
    icon: alertIcon,
    cancelButtonColor: theme.palette.primary.main,
    confirmButtonColor: theme.palette.error.main,
    showCancelButton: true,
    heightAuto: false,
    confirmButtonText: alertConfirmButtonText,
    cancelButtonText: alertCancelButtonText,
  }).then((modalState) => {
    if (modalState.isConfirmed) {
      !!confirmCallback && confirmCallback();
    } else {
      !!cancelCallback && cancelCallback();
    }
  });
};

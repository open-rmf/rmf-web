import { createMuiTheme } from '@material-ui/core';
import Swal, { SweetAlertIcon, SweetAlertOptions } from 'sweetalert2';

const theme = createMuiTheme();

export interface VerificationAlertProps {
  title?: string;
  body?: string;
  icon?: SweetAlertIcon;
  confirmButtonText?: string;
  cancelButtonText?: string;
  confirmCallback?: () => void;
  cancelCallback?: () => void;
}

export class Alerts {
  static success = (message?: string) => {
    const swalParams = Alerts.getSuccessMsg(message);
    return Swal.fire(swalParams);
  };

  static error = (message?: string) => {
    const swalParams = Alerts.getErrorMsg(message);
    return Swal.fire(swalParams);
  };

  /**
   * Modal to warn a user about something related to an action.
   */
  static verification = (props: VerificationAlertProps) => {
    const { confirmCallback, cancelCallback } = props;
    const verificationParams = Alerts.getVerificationParams(props);
    Swal.fire(verificationParams).then((modalState) => {
      if (modalState.isConfirmed) {
        !!confirmCallback && confirmCallback();
      } else {
        !!cancelCallback && cancelCallback();
      }
    });
  };

  static getSuccessMsg = (message?: string): SweetAlertOptions => {
    return {
      title: 'Done!',
      text: !!message ? `${message}` : 'Successful Operation',
      icon: 'success',
      timer: 2000,
      heightAuto: false,
    };
  };

  static getErrorMsg = (message?: string): SweetAlertOptions => {
    return {
      title: 'Ups',
      text: !!message ? `${message}` : 'An error has occurred',
      icon: 'error',
      heightAuto: false,
    };
  };

  static getVerificationParams = (props: VerificationAlertProps): SweetAlertOptions => {
    const { title, body, icon, confirmButtonText, cancelButtonText } = props;
    const alertTitle = title || 'Are you sure you want to continue?';
    const alertText = body || 'Once you accept this there is no turning back.';
    const alertIcon = icon || 'warning';
    const alertConfirmButtonText = confirmButtonText || `Yes, I'm sure.`;
    const alertCancelButtonText = cancelButtonText || 'No';

    return {
      title: alertTitle,
      text: alertText,
      icon: alertIcon,
      cancelButtonColor: theme.palette.primary.main,
      confirmButtonColor: theme.palette.error.main,
      showCancelButton: true,
      heightAuto: false,
      confirmButtonText: alertConfirmButtonText,
      cancelButtonText: alertCancelButtonText,
    };
  };
}

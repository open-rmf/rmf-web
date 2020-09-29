import Swal from 'sweetalert2';

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

import Swal from 'sweetalert2';
import { verificationAlert } from '../alerts';

test('Render with default values', async () => {
  // MySwal.fire('hue');
  // console.log(MySwal)
  const hue = verificationAlert({});
  console.log(hue);

  // expect(Swal.isVisible()).toBeTruthy();
  // console.log(window)
  // console.log(window['swal2-title'])
  // // spyOn(Swal, "fire");
  // const root = mount(await)
  // expect(root).toMatchSnapshot()
});

const globalAny: any = global;

import Enzyme from 'enzyme';
import Adapter from 'enzyme-adapter-react-16';

Enzyme.configure({ adapter: new Adapter() });

class MockEncoder {
  encode() {
    return jest.fn();
  }
}
const mockCrypto = { subtle: { digest: () => jest.fn() } };
globalAny.window.TextEncoder = MockEncoder;
globalAny.window.crypto = mockCrypto;

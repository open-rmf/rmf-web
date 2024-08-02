import { beforeEach, describe, it, vi } from 'vitest';

import { SioClient } from '.';

describe('subscriptions', () => {
  let sioClient;

  beforeEach(() => {
    sioClient = new SioClient('', { autoConnect: false });
    vi.spyOn(sioClient.sio, 'emit');
  });

  it('dummy', () => {
    // Dummy test to ci passes.
    // #940 removed multiplexing in order to support resubscribe on reconnect.
    // With it gone, there is no more tests and ci fails as a result.
  });
});

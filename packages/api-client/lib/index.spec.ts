import { SioClient } from '.';

describe('subscriptions', () => {
  let sioClient: SioClient;

  beforeEach(() => {
    sioClient = new SioClient({ autoConnect: false });
    spyOn(sioClient.sio, 'emit');
  });

  it('multiplexes subscriptions', () => {
    const s1 = sioClient.subscribe('test', () => {
      // empty
    });
    const s2 = sioClient.subscribe('test', () => {
      // empty
    });
    expect(sioClient.sio.emit).toHaveBeenCalledOnceWith('subscribe', jasmine.anything());
    (sioClient.sio.emit as jasmine.Spy).calls.reset();

    sioClient.unsubscribe(s1);
    sioClient.unsubscribe(s2);
    expect(sioClient.sio.emit).toHaveBeenCalledOnceWith('unsubscribe', jasmine.anything());
  });

  it('does not unsubscribe early when there are multiple subscriptions with the same listener', () => {
    const listener = jasmine.createSpy();
    const s1 = sioClient.subscribe('test', listener);
    const s2 = sioClient.subscribe('test', listener);
    expect(sioClient.sio.emit).toHaveBeenCalledOnceWith('subscribe', jasmine.anything());
    (sioClient.sio.emit as jasmine.Spy).calls.reset();

    sioClient.unsubscribe(s1);
    expect(sioClient.sio.emit).not.toHaveBeenCalled();
    sioClient.unsubscribe(s2);
    expect(sioClient.sio.emit).toHaveBeenCalledOnceWith('unsubscribe', jasmine.anything());
  });
});

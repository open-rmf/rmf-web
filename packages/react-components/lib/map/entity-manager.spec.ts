import { BBox } from 'rbush';
import { EntityManager } from './entity-manager';

describe('EntityManager - getNonColliding', () => {
  it('returns original bbox if it does not collide', () => {
    const entityManager = new EntityManager();
    entityManager.add({ bbox: { minX: 0, minY: 0, maxX: 100, maxY: 100 } });
    const originalBBox = {
      minX: 110,
      minY: 110,
      maxX: 200,
      maxY: 200,
    };
    const nonCollidingBBox = entityManager.getNonColliding(originalBBox);
    expect(nonCollidingBBox).toBe(originalBBox);
  });

  it('returns non-colliding result', () => {
    const entityManager = new EntityManager();
    entityManager.add({ bbox: { minX: 0, minY: 0, maxX: 100, maxY: 100 } });
    const nonCollidingBBox: BBox = entityManager.getNonColliding({
      minX: 90,
      minY: 100,
      maxX: 110,
      maxY: 120,
    }) as BBox;
    expect(nonCollidingBBox).not.toBeNull();
    expect(entityManager.collides(nonCollidingBBox)).toBe(false);
  });
});

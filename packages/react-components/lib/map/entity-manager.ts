import RBush, { BBox } from 'rbush';
import React from 'react';

export type { BBox } from 'rbush';

export interface Entity {
  bbox: BBox;
}

class EntityRBush extends RBush<Entity> {
  toBBox(entity: Entity): BBox {
    return entity.bbox;
  }
}

export interface GetNonCollidingOptions {
  searchDepth?: number;
  searchLimit?: number;
  distLimit?: number;
}

export class EntityManager {
  add(entity: Entity): Entity {
    this._entities.insert(entity);
    return entity;
  }

  remove(entity: Entity): void {
    this._entities.remove(entity);
  }

  /**
   * Get a bounding box that does not collides with any other entities.
   * @param bbox Preferred location.
   * @param searchDepth
   * @returns
   */
  getNonColliding(
    bbox: BBox,
    { searchDepth = 4, distLimit = 200 }: GetNonCollidingOptions = {},
  ): BBox | null {
    const distLimitSq = distLimit ** 2;
    const candidates: BBox[] = [];
    const stack: { bbox: BBox; depth: number }[] = [{ bbox, depth: 0 }];

    for (let top = stack.pop(); top !== undefined; top = stack.pop()) {
      const { bbox: search, depth } = top;
      const collidingEntites = this._entities.search(search);
      const distSq = (search.minX - bbox.minX) ** 2 + (search.minY - bbox.minY) ** 2;
      if (collidingEntites.length === 0 && distSq <= distLimitSq) {
        candidates.push(search);
      } else if (depth < searchDepth) {
        collidingEntites.forEach((colliding) => {
          // left
          stack.push({
            bbox: {
              minX: bbox.minX - (bbox.maxX - colliding.bbox.minX + 0.001),
              minY: bbox.minY,
              maxX: colliding.bbox.minX - 0.001,
              maxY: bbox.maxY,
            },
            depth: depth + 1,
          });
          // right
          stack.push({
            bbox: {
              minX: colliding.bbox.maxX + 0.001,
              minY: bbox.minY,
              maxX: bbox.maxX + (colliding.bbox.maxX - bbox.minX + 0.001),
              maxY: bbox.maxY,
            },
            depth: depth + 1,
          });
          // top
          stack.push({
            bbox: {
              minX: bbox.minX,
              minY: bbox.minY - (bbox.maxY - colliding.bbox.minY + 0.001),
              maxX: bbox.maxX,
              maxY: colliding.bbox.minY - 0.001,
            },
            depth: depth + 1,
          });
          // bottom
          stack.push({
            bbox: {
              minX: bbox.minX,
              minY: colliding.bbox.maxY + 0.001,
              maxX: bbox.maxX,
              maxY: bbox.maxY + (colliding.bbox.maxY - bbox.minY + 0.001),
            },
            depth: depth + 1,
          });
        });
      }
    }

    if (candidates.length === 0) return null;

    const preferredCenterX = bbox.minX + (bbox.maxX - bbox.minX) / 2;
    const preferredCenterY = bbox.minY + (bbox.maxY - bbox.minY) / 2;
    const distances = candidates.map((candidate) => {
      const centerX = candidate.minX + (candidate.maxX - candidate.minX) / 2;
      const centerY = candidate.minY + (candidate.maxY - candidate.minY) / 2;
      return (preferredCenterY - centerY) ** 2 + (preferredCenterX - centerX) ** 2;
    });
    let closestIdx = 0;
    let closestDistance = distances[0];
    distances.forEach((d, i) => {
      if (d < closestDistance) {
        closestDistance = d;
        closestIdx = i;
      }
    });
    return candidates[closestIdx];
  }

  /**
   * Returns true if the bounding box collidies with any other entities.
   * @param bbox
   * @returns
   */
  collides(bbox: BBox): boolean {
    return this._entities.collides(bbox);
  }

  private _entities = new EntityRBush();
}

export const EntityManagerContext = React.createContext(new EntityManager());

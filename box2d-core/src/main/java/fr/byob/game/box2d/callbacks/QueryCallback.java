package fr.byob.game.box2d.callbacks;

import fr.byob.game.box2d.dynamics.Fixture;

public interface QueryCallback {

	boolean reportFixture(Fixture fixture);

}

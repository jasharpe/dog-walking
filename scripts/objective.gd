extends Resource
class_name Objective

signal updated

@export var text: String
@export var total: int
var done: int:
	set(new_done):
		done = new_done
		updated.emit()
var complete: bool:
	get():
		return done == total

#include "python_task.hpp"

thread_local PyThreadState* PythonThreadState::state = nullptr;

void PythonTask::run_till_completion()
{
    auto& sched = sync_scheduler();

    while (!this->ready()) {
        sched.run_all_tasks();
    }
}

void create_task_module(py::module& m)
{

    using arg = py::arg;

    py::enum_<TaskMode>(m, "TaskMode")
        .value("Async", TaskMode::Async)
        .value("Sync", TaskMode::Sync)
        .value("Inline", TaskMode::Inline);

    py::class_<PythonTask>(m, "Task")
        .def(py::init<py::object, py::str, TaskMode>(), arg("function"), arg("name"), arg("mode") = TaskMode::Sync)
        .def("then", &PythonTask::then_python,
            arg("task"),
            arg("TaskMode") = TaskMode::Sync,
            py::return_value_policy::reference_internal)
        .def("ready", &PythonTask::ready)
        .def("get", &PythonTask::get)
        .def("run_till_completion", &PythonTask::run_till_completion);
}